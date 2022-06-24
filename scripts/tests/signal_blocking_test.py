import signal

#based in part on https://github.com/ossobv/pysigset/blob/master/pysigset.py
#The primary limitation of this approach is that it can only 'hold' a single signal
class block_signals(object):
    """
    Suspend the supplied signals in the 'with' block.
    Usage:
        with block_signals(SIGINT, SIGTERM):
            # Signals are blocked here...
            pass
    """

    def __init__(self, *signals):
        self.blocked_signals = signals if len(signals) > 0  else signal.valid_signals()
        self.old_signal_mask = None

    def __enter__(self):
        self.old_signal_mask = signal.pthread_sigmask(signal.SIG_BLOCK, self.blocked_signals)

    def __exit__(self, type, value, tb):
        signal.pthread_sigmask(signal.SIG_SETMASK, self.old_signal_mask)




#Downloaded from https://gist.github.com/evansd/2375136
# The MIT License (MIT)
#
# Copyright (c) 2013 David Evans
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import os
import signal


class defer_signals(object):
    """
    Context manager to defer signal handling until context exits.
    Takes optional list of signals to defer (default: SIGHUP, SIGINT, SIGTERM).
    Signals can be identified by number or by name.
    Allows you to wrap instruction sequences that ought to be atomic and ensure
    that they don't get interrupted mid-way.
    """

    def __init__(self, signal_list=None):
        # Default list of signals to defer
        if signal_list is None:
            signal_list = [signal.SIGHUP, signal.SIGINT, signal.SIGTERM]
        # Accept either signal numbers or string identifiers
        self.signal_list = [
            getattr(signal, sig_id) if isinstance(sig_id, str) else sig_id
            for sig_id in signal_list
        ]
        self.deferred = []
        self.previous_handlers = {}

    def defer_signal(self, sig_num, stack_frame):
        self.deferred.append(sig_num)

    def __enter__(self):
        # Replace existing handlers with deferred handler
        for sig_num in self.signal_list:
            # signal.signal returns None when no handler has been set in Python,
            # which is the same as the default handler (SIG_DFL) being set
            self.previous_handlers[sig_num] = (
                signal.signal(sig_num, self.defer_signal) or signal.SIG_DFL)
        return self

    def __exit__(self, *args):
        # Restore handlers
        for sig_num, handler in self.previous_handlers.items():
            signal.signal(sig_num, handler)
        # Send deferred signals
        while self.deferred:
            sig_num = self.deferred.pop(0)
            os.kill(os.getpid(), sig_num)

    def __call__(self):
        """
        If there are any deferred signals pending, trigger them now
        This means that instead of this code:
            for item in collection:
                with defer_signals():
                    item.process()
        You can write this:
            with defer_signals() as handle_signals:
                for item in collection:
                    item.process()
                    handle_signals()
        Which has the same effect but avoids having to embed the context
        manager in the loop
        """
        if self.deferred:
            # Reattach the signal handlers and fire signals
            self.__exit__()
            # Put our deferred signal handlers back in place
            self.__enter__()





import time

def run_test(blocker, name):

    signals = (signal.SIGINT, signal.SIGTERM)
    global counter
    counter = 0

    def sighandler(signum, frame):
        global counter
        counter += 1
        print(str(name) + ": " + str(counter))

    for signum in signals:
        signal.signal(signum, sighandler)

    while counter < 20:
        with blocker():
            print("Blocking!")
            time.sleep(1)
        print("Not blocking!")
        time.sleep(1)


def run_tests():
    import multiprocessing as mp

    a = mp.Process(target=run_test, args=[block_signals, 'blocker'])
    b = mp.Process(target=run_test, args=[defer_signals, 'deferer'])
    a.start()
    b.start()

    with block_signals():
        a.join()
        b.join()


if __name__=='__main__':
    run_tests()
