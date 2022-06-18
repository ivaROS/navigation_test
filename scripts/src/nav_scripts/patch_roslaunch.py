from roslaunch.nodeprocess import _logger, LocalProcess, get_ros_root, FatalProcessLaunch
from roslaunch.core import printerrlog, printlog_bold

import errno
import os
import signal
import subprocess
import traceback
import rospkg

def start(self):
    """
    Start the process.

    @raise FatalProcessLaunch: if process cannot be started and it
    is not likely to ever succeed
    """
    #print("YES WE ARE USING MY CODE!")
    super(LocalProcess, self).start()
    try:
        self.lock.acquire()
        if self.started:
            _logger.info("process[%s]: restarting os process", self.name)
        else:
            _logger.info("process[%s]: starting os process", self.name)
        self.started = self.stopped = False

        full_env = self.env

        # _configure_logging() can mutate self.args
        try:
            logfileout, logfileerr = self._configure_logging()
        except Exception as e:
            _logger.error(traceback.format_exc())
            printerrlog("[%s] ERROR: unable to configure logging [%s] " %(self.name, str(e)))
            # it's not safe to inherit from this process as
            # rostest changes stdout to a StringIO, which is not a
            # proper file.
            logfileout, logfileerr = subprocess.PIPE, subprocess.PIPE

        if self.cwd == 'node':
            cwd = os.path.dirname(self.args[0])
        elif self.cwd == 'cwd':
            cwd = os.getcwd()
        elif self.cwd == 'ros-root':
            cwd = get_ros_root()
        else:
            cwd = rospkg.get_ros_home()
        if not os.path.exists(cwd):
            try:
                os.makedirs(cwd)
            except OSError:
                # exist_ok=True
                pass

        _logger.info("process[%s]: start w/ args [%s]", self.name, self.args)
        _logger.info("process[%s]: cwd will be [%s]", self.name, cwd)

        try:
            _ = os.setsid
            close_file_descriptor = True
        except AttributeError:
            close_file_descriptor = False

        def preexec_function():
            if close_file_descriptor:
                os.setsid()
            #from https://stackoverflow.com/a/5050521
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)

        try:
            self.popen = subprocess.Popen(self.args, cwd=cwd, stdout=logfileout, stderr=logfileerr, env=full_env, close_fds=close_file_descriptor, preexec_fn=preexec_function)
        except OSError as e:
            self.started = True # must set so is_alive state is correct
            _logger.error("OSError(%d, %s)", e.errno, e.strerror)
            if e.errno == errno.ENOEXEC:  # Exec format error
                raise FatalProcessLaunch \
                    ("Unable to launch [%s]. \nIf it is a script, you may be missing a '#!' declaration at the top. " %self.name)
            elif e.errno == errno.ENOENT:  # no such file or directory
                raise FatalProcessLaunch("""Roslaunch got a '%s' error while attempting to run:

%s

Please make sure that all the executables in this command exist and have
executable permission. This is often caused by a bad launch-prefix.""" %(e.strerror, ' '.join(self.args)))
            else:
                raise FatalProcessLaunch("unable to launch [%s]: %s " %(' '.join(self.args), e.strerror))

        self.started = True
        # Check that the process is either still running (poll returns
        # None) or that it completed successfully since when we
        # launched it above (poll returns the return code, 0).
        poll_result = self.popen.poll()
        if poll_result is None or poll_result == 0:
            self.pid = self.popen.pid
            printlog_bold("process[%s]: started with pid [%s] " %(self.name, self.pid))
            return True
        else:
            printerrlog("failed to start local process: %s " %(' '.join(self.args)))
            return False
    finally:
        self.lock.release()

# from https://stackoverflow.com/a/10028797
#LocalProcess.__dict__["start"] = start
LocalProcess.start = start

from roslaunch.loader import convert_value, Loader
import sys

def param_value(self, verbose, name, ptype, value, textfile, binfile, command):
    print("MY PARAM CODE IS ALSO CALLED")
    """
    Parse text representation of param spec into Python value
    @param name: param name, for error message use only
    @type  name: str
    @param verbose: print verbose output
    @type  verbose: bool
    @param textfile: name of text file to load from, or None
    @type  textfile: str
    @param binfile: name of binary file to load from, or None
    @type  binfile: str
    @param command: command to execute for parameter value, or None
    @type  command: str
    @raise ValueError: if parameters are invalid
    """
    if value is not None:
        return convert_value(value.strip(), ptype)
    elif textfile is not None:
        with open(textfile, 'r') as f:
            return convert_value(f.read(), ptype)
    elif binfile is not None:
        try:
            from xmlrpc.client import Binary
        except ImportError:
            from xmlrpclib import Binary
        with open(binfile, 'rb') as f:
            return Binary(f.read())
    elif command is not None:
        try:
            if type(command) == unicode:
                command = command.encode('utf-8')  # attempt to force to string for shlex/subprocess
        except NameError:
            pass
        if verbose:
            print("... executing command param [%s]" % command)
        import subprocess, shlex  # shlex rocks
        try:
            if os.name != 'nt':
                command = shlex.split(command)
            else:
                cl = shlex.split(command, posix=False)  # use non-posix method on Windows

                # On Linux, single quotes are commonly used to enclose a path to escape spaces.
                # However, on Windows, the single quotes are treated as part of the arguments.
                # Special handling is required to remove the extra single quotes.
                if "'" in command:
                    cl = [token[1:-1] if token.startswith("'") and token.endswith("'") else token for token in cl]
                command = cl

                # Python scripts in ROS tend to omit .py extension since they could become executable with shebang line
                # special handle the use of Python scripts in Windows environment:
                # 1. search for a wrapper executable (of the same name) under the same directory with stat.S_IXUSR flag
                # 2. if no wrapper is present, prepend command with 'python' executable
                if os.path.isabs(cl[0]):
                    # trying to launch an executable from a specific location(package), e.g. xacro
                    import stat
                    rx_flag = stat.S_IRUSR | stat.S_IXUSR
                    if not os.path.exists(cl[0]) or os.stat(cl[0]).st_mode & rx_flag != rx_flag:
                        d = os.path.dirname(cl[0])
                        files_of_same_name = [
                            os.path.join(d, f) for f in os.listdir(d)
                            if os.path.splitext(f)[0].lower() == os.path.splitext(os.path.basename(cl[0]))[0].lower()
                        ] if os.path.exists(d) else []
                        executable_command = None
                        for f in files_of_same_name:
                            if os.stat(f).st_mode & rx_flag == rx_flag:
                                # found an executable wrapper of the desired Python script
                                executable_command = f

                        if not executable_command:
                            for f in files_of_same_name:
                                mode = os.stat(f).st_mode
                                if (mode & stat.S_IRUSR == stat.S_IRUSR) and (mode & stat.S_IXUSR != stat.S_IXUSR):
                                    # when there is read permission but not execute permission, this is typically a Python script (in ROS)
                                    if os.path.splitext(f)[1].lower() in ['.py', '']:
                                        executable_command = ' '.join([sys.executable, f])
                        if executable_command:
                            command[0] = executable_command

            def preexec_function():
                # from https://stackoverflow.com/a/5050521
                signal.signal(signal.SIGINT, signal.SIG_IGN)
                signal.signal(signal.SIGTERM, signal.SIG_IGN)

            p = subprocess.Popen(command, stdout=subprocess.PIPE, preexec_fn=preexec_function)
            c_value = p.communicate()[0]
            if not isinstance(c_value, str):
                c_value = c_value.decode('utf-8')
            if p.returncode != 0:
                raise ValueError("Cannot load command parameter [%s]: command [%s] returned with code [%s]" % (
                name, command, p.returncode))
        except OSError as e:
            if e.errno == errno.ENOENT:
                raise ValueError("Cannot load command parameter [%s]: no such command [%s]" % (name, command))
            raise
        if c_value is None:
            raise ValueError("parameter: unable to get output of command [%s]" % command)
        return convert_value(c_value, ptype)
    else:  # _param_tag prevalidates, so this should not be reachable
        raise ValueError("unable to determine parameter value")

#p = subprocess.Popen(command, stdout=subprocess.PIPE)
Loader.param_value = param_value