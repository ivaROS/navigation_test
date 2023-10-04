class RosLauncherException(BaseException):
    pass


class RosLauncherHelperTest(object):

    @classmethod
    def init(cls):
        class RosLauncherRuntimeException(RosLauncherException):
            pass

        class RosLauncherLaunchException(RosLauncherException):
            pass

        cls.exc_type_launch = RosLauncherLaunchException
        cls.exc_type_runtime = RosLauncherRuntimeException

    def launch(self, error):
        if error:
            raise type(self).exc_type_launch
        else:
            print(str(type(self)) +  ": Launched successfully")

    def run(self, error):
        if error:
            raise type(self).exc_type_runtime
        else:
            print(str(type(self)) + ": run successfully")

    def shutdown(self):
        print(str(type(self)) + ": Shutdown")


class Test1(RosLauncherHelperTest):
    pass

Test1.init()


class Test2(RosLauncherHelperTest):
    pass

Test2.init()


class LauncherContextManager(object):

    def __init__(self, launcher):
        self.launcher = launcher

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type == type(self.launcher).exc_type_launch or exc_type == type(self.launcher).exc_type_runtime:
            self.launcher.shutdown()
            return True
        pass


def run_test():
    outer_launcher = Test1()
    inner_launcher = Test2()

    for code in range(-1,5):
        print("Code: " + str(code))
        with LauncherContextManager(outer_launcher):
            outer_launcher.launch(error=(code==0))

            with LauncherContextManager(inner_launcher):
                inner_launcher.launch(error=(code==1))

                outer_launcher.run(error=(code==2))
                inner_launcher.run(error=(code==3))

if __name__=="__main__":
    run_test()