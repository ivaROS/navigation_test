import multiprocessing as mp
import threading
import time


def do_stuff():
    s = 0
    for i in range(int(1e6)):
        s+=i


def run_basic_test(num):

    class Worker(mp.Process):

        def __init__(self, num=0):
            super(Worker, self).__init__()
            self.num = num
            pass

        def run(self):
            while not Coordinator.shutdown_event.is_set():
                print("[" + str(self.num) + "]: Do some work")
                do_stuff()
            print("[" + str(self.num) + "]: Exiting!")


    class Coordinator(object):
        shutdown_event = mp.Event()

        def __init__(self):
            Coordinator.shutdown_event.clear()

        def shutdown(self):
            Coordinator.shutdown_event.set()


    coord = Coordinator()
    processes = [Worker(i) for i in range(num)]
    print("Created [" + str(num) + "] Workers")

    for p in processes:
        p.start()

    print("Started Workers, waiting to join")

    time.sleep(5)

    print("Sending shutdown signal...")
    coord.shutdown()

    for p in processes:
        p.join()

    print("All Done")



def run_queue_test(num):

    import threading
    import queue
    import types


    class TaskCommunication(object):

        #Takes in lists (or generators) of tasks and transfers them, in order, to the task queue
        class TaskQueuer(queue.Queue):

            def __init__(self, task_queue, shutdown_event):
                self.task_queue = task_queue
                self.thread = threading.Thread(target=self.run)
                self.task_input_queue = queue.Queue(10)
                self.shutdown_event = shutdown_event
                self.thread.start()

            def run(self):
                is_empty = False
                is_full = False

                while not self.shutdown_event.is_set():
                    try:
                        task_obj = self.task_input_queue.get(block=False)
                    except queue.Empty as e:
                        if not is_empty:
                            print("Task input queue is empty, will check again later")
                        is_empty = True
                        time.sleep(1)
                    else:
                        is_empty = False

                        task_it = task_obj() if isinstance(task_obj, types.GeneratorType) else task_obj if isinstance(
                            task_obj, list) else None

                        for t in task_it:
                            while not self.shutdown_event.is_set():
                                try:
                                    self.task_queue.put(t, block=False)
                                except queue.Full as e:
                                    if not is_full:
                                        print("Task queue is full!, will try to add task later")
                                    is_full = True
                                    time.sleep(1)
                                else:
                                    is_full = False
                                    break

                print("Shutting down task queuer!")

            def add_tasks(self, tasks):
                is_full = False
                while not self.shutdown_event.is_set():
                    try:
                        self.task_input_queue.put(item=tasks, block=False)
                    except queue.Full as e:
                        if not is_full:
                            print("Task input queue is full!, will try to add task later")
                        is_full = True
                        time.sleep(1)
                    else:
                        break

        def __init__(self):
            self.task_queue = mp.JoinableQueue(maxsize=5)
            self.result_queue = mp.JoinableQueue(maxsize=5)
            self.shutdown_event = mp.Event()
            self.task_adder = TaskCommunication.TaskQueuer(task_queue=self.task_queue, shutdown_event=self.shutdown_event)

        def shutdown(self):
            self.shutdown_event.set()

        def wait_to_finish(self):
            pass

        def add_tasks(self, tasks):
            self.task_adder.add_tasks(tasks=tasks)


    class Worker(mp.Process):

        def __init__(self, tc, num=0):
            super(Worker, self).__init__()

            self.tc = tc
            self.num = num


        def run(self):
            is_empty = False
            while not self.tc.shutdown_event.is_set():
                try:
                    task = self.tc.task_queue.get(block=False)
                except queue.Empty as e:
                    if not is_empty:
                        print("No task in task queue!")
                    is_empty = True
                    time.sleep(1)
                else:
                    is_empty = False
                    msg = "[" + str(self.num) + "]: Do some work (" + str(task["index"])
                    print(msg)
                    do_stuff()
                    #self.tc.result_queue.put
                    self.tc.task_queue.task_done()

            print("[" + str(self.num) + "]: Exiting!")

    def make_tasks_gen(num):
        for i in range(num):
            yield {"index": i}

    def make_tasks_list(num):
        return [i for i in make_tasks_gen(num=num)]


    tc = TaskCommunication()

    processes = [Worker(tc=tc, num=i) for i in range(num)]
    print("Created [" + str(num) + "] Workers")

    for p in processes:
        p.start()

    for _ in range(3):
        tc.add_tasks(make_tasks_list(num=10))

    print("Started Workers, waiting to join")

    time.sleep(20)

    print("Sending shutdown signal...")
    tc.shutdown()

    for p in processes:
        p.join()

    print("All Done")




def run_queue_wrapper_test(num):

    import threading
    import queue
    import types
    import signal


    class GracefulShutdownException(BaseException): pass

    class InterruptibleQueueWrapper(object):

        def __init__(self, queue, shutdown_event, sleep_time=1, name="queue"):
            self.queue = queue
            self.shutdown_event = shutdown_event
            self.sleep_time = sleep_time
            self.name = name

        def put(self, task):
            is_full = False
            while not self.shutdown_event.is_set():
                try:
                    self.queue.put(task, block=True, timeout=self.sleep_time)
                except queue.Full as e:
                    if not is_full:
                        print("[" + str(self.name) + "] is full!, unable to add task!")
                    is_full = True
                else:
                    print("Added task " + str(task) + " to [" + str(self.name) + "]")
                    break

        def get(self):

            class NextGetter(object):

                def __enter__(myself):
                    is_empty = False

                    while True:
                        try:
                            task = self.queue.get(block=True, timeout=self.sleep_time)
                        except queue.Empty as e:
                            if not is_empty:
                                print("No task in [" + str(self.name) + "]!")
                            is_empty = True
                            if self.shutdown_event.is_set():
                                raise GracefulShutdownException()
                        else:
                            print("Got task " + str(task) + " from [" + str(self.name) + "]!")
                            return task

                    raise GracefulShutdownException()

                def __exit__(myself, exc_type, exc_val, exc_tb):
                    #if exc_type is not None and issubclass(exc_type, NextGetter.DoneException):
                    #    return True
                    self.queue.task_done()

            return NextGetter()

        def join(self):
            return self.queue.join()

    class TaskCommunication(object):

        #Takes in lists (or generators) of tasks and transfers them, in order, to the task queue
        class TaskQueuer(object):

            def __init__(self, task_queue, shutdown_event):
                self.task_queue = task_queue #InterruptibleQueueWrapper(queue=task_queue, shutdown_event=shutdown_event, sleep_time=1, name="Task Queue")
                self.thread = threading.Thread(target=self.run)
                self.task_input_queue = InterruptibleQueueWrapper(queue=queue.Queue(10), shutdown_event=shutdown_event, sleep_time=1, name="Task Input Queue")
                self.shutdown_event = shutdown_event
                self.thread.start()

            def run(self):

                try:
                    while True:
                        with self.task_input_queue.get() as task_obj:
                            task_it = task_obj if isinstance(task_obj, types.GeneratorType) else task_obj if isinstance(task_obj, list) else None
                            #Only pass tasks on if not trying to shutdown
                            if not self.shutdown_event.is_set():
                                for t in task_it:
                                    self.task_queue.put(t)
                            else:
                                print("Task input queue: Ignoring tasks " + str(task_it))

                except GracefulShutdownException as e:
                    print(str(e))

                print("Shutting down task queuer!")


            def add_tasks(self, tasks):
                self.task_input_queue.put(task=tasks)

            def wait_to_finish(self):
                print("Wait for task input queue to join...")
                self.task_input_queue.queue.join()
                print("Task input queue joined!")


        def __init__(self):
            self.shutdown_event = mp.Event()

            self.task_queue = InterruptibleQueueWrapper(queue=mp.JoinableQueue(maxsize=10), shutdown_event=self.shutdown_event, sleep_time=1,
                                                        name="Task Queue")

            self.result_queue = mp.JoinableQueue(maxsize=10)
            self.task_adder = TaskCommunication.TaskQueuer(task_queue=self.task_queue, shutdown_event=self.shutdown_event)

        def shutdown(self):
            self.shutdown_event.set()

        def wait_to_finish(self):
            self.task_adder.wait_to_finish()

            print("Wait for task queue to join...")
            self.task_queue.join()
            print("Task queue joined!")
            self.shutdown()

        def add_tasks(self, tasks):
            self.task_adder.add_tasks(tasks=tasks)


    class Worker(mp.Process):

        def __init__(self, tc, num=0):
            super(Worker, self).__init__()
            self.tc = tc
            self.num = num

        def signal_handler(self, signum, frame):
            print("[" + str(self.num) + "]: Received signal " + str(signum)) # + "\n\n" + str(frame))

        def run(self):
            signal.signal(signal.SIGINT, self.signal_handler)
            signal.signal(signal.SIGTERM, self.signal_handler)

            try:
                while True: #not self.tc.shutdown_event.is_set():
                    with self.tc.task_queue.get() as task:
                        if not self.tc.shutdown_event.is_set():
                            msg = "[" + str(self.num) + "]: Do some work (" + str(task)
                            print(msg)
                            do_stuff()
                        else:
                            print("[" + str(self.num) + "]: Ignoring task " + str(task))

            except GracefulShutdownException as e:
                print(str(e))

            print("[" + str(self.num) + "]: Exiting!")

    def make_tasks_gen(num):
        for i in range(num):
            yield {"index": i}

    def make_tasks_list(num):
        return [i for i in make_tasks_gen(num=num)]


    tc = TaskCommunication()

    processes = [Worker(tc=tc, num=i) for i in range(num)]
    print("Created [" + str(num) + "] Workers")

    def shutdown():
        tc.shutdown()

    def signal_handler(signum, frame):
        print("Main process received signal " + str(signum))
        #print("Main process: \n\n" + str(signum) + "\n\n" + str(frame))
        shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    #tc.add_tasks(make_tasks_list(num=30))
    for _ in range(5):
        tc.add_tasks(make_tasks_list(num=100))

    for p in processes:
        p.start()
    print("Started Workers")

    tc.add_tasks(make_tasks_gen(num=500))


    if False:
        time.sleep(20)
    else:
        tc.wait_to_finish()

    print("Sending shutdown signal...")
    tc.shutdown()

    for p in processes:
        p.join()

    print("All Done")




def interruptible_queue_test(num):

    import threading
    import queue
    import types
    import signal


    class GracefulShutdownException(BaseException): pass

    def InterruptibleQueue(maxsize=0):
        import multiprocessing as mp
        import multiprocessing.queues

        class InterruptibleQueueImpl(multiprocessing.queues.JoinableQueue):

            def join(self, timeout=None):
                with self._cond:
                    if not self._unfinished_tasks._semlock._is_zero():
                        return self._cond.wait(timeout=timeout)

        '''Returns a queue object'''
        return InterruptibleQueueImpl(maxsize, ctx=mp.get_context())


    class InterruptibleQueueWrapper(object):

        def __init__(self, queue, shutdown_event, sleep_time=1, name="queue"):
            self.queue = queue
            self.shutdown_event = shutdown_event
            self.sleep_time = sleep_time
            self.name = name

        def put(self, task):
            is_full = False
            while not self.shutdown_event.is_set():
                try:
                    self.queue.put(task, block=True, timeout=self.sleep_time)
                except queue.Full as e:
                    if not is_full:
                        print("[" + str(self.name) + "] is full!, unable to add task!")
                    is_full = True
                else:
                    print("Added task " + str(task) + " to [" + str(self.name) + "]")
                    break

        def get(self):

            class NextGetter(object):

                def __enter__(myself):
                    is_empty = False

                    while True:
                        try:
                            task = self.queue.get(block=True, timeout=self.sleep_time)
                        except queue.Empty as e:
                            if not is_empty:
                                print("No task in [" + str(self.name) + "]!")
                            is_empty = True
                            if self.shutdown_event.is_set():
                                raise GracefulShutdownException()
                        else:
                            print("Got task " + str(task) + " from [" + str(self.name) + "]!")
                            return task


                def __exit__(myself, exc_type, exc_val, exc_tb):
                    #if exc_type is not None and issubclass(exc_type, NextGetter.DoneException):
                    #    return True
                    self.queue.task_done()

            return NextGetter()

        def join(self):
            return self.queue.join()

    class TaskCommunication(object):

        #Takes in lists (or generators) of tasks and transfers them, in order, to the task queue
        class TaskQueuer(object):

            def __init__(self, task_queue, shutdown_event):
                self.task_queue = task_queue #InterruptibleQueueWrapper(queue=task_queue, shutdown_event=shutdown_event, sleep_time=1, name="Task Queue")
                self.thread = threading.Thread(target=self.run)
                self.task_input_queue = InterruptibleQueueWrapper(queue=queue.Queue(10), shutdown_event=shutdown_event, sleep_time=1, name="Task Input Queue")
                self.shutdown_event = shutdown_event
                self.thread.start()

            def run(self):

                try:
                    while True:
                        with self.task_input_queue.get() as task_obj:
                            task_it = task_obj if isinstance(task_obj, types.GeneratorType) else task_obj if isinstance(task_obj, list) else None
                            #Only pass tasks on if not trying to shutdown
                            if not self.shutdown_event.is_set():
                                for t in task_it:
                                    self.task_queue.put(t)
                            else:
                                print("Task input queue: Ignoring tasks " + str(task_it))

                except GracefulShutdownException as e:
                    print(str(e))

                print("Shutting down task queuer!")


            def add_tasks(self, tasks):
                self.task_input_queue.put(task=tasks)

            def wait_to_finish(self):
                print("Wait for task input queue to join...")
                self.task_input_queue.queue.join()
                print("Task input queue joined!")


        def __init__(self):
            self.shutdown_event = mp.Event()

            self.task_queue = InterruptibleQueueWrapper(queue=mp.JoinableQueue(maxsize=10), shutdown_event=self.shutdown_event, sleep_time=1,
                                                        name="Task Queue")

            self.result_queue = mp.JoinableQueue(maxsize=10)
            self.task_adder = TaskCommunication.TaskQueuer(task_queue=self.task_queue, shutdown_event=self.shutdown_event)

        def shutdown(self):
            self.shutdown_event.set()

        def wait_to_finish(self):
            self.task_adder.wait_to_finish()

            print("Wait for task queue to join...")
            self.task_queue.join()
            print("Task queue joined!")
            self.shutdown()

        def add_tasks(self, tasks):
            self.task_adder.add_tasks(tasks=tasks)


    class Worker(mp.Process):

        def __init__(self, tc, num=0):
            super(Worker, self).__init__()
            self.tc = tc
            self.num = num

        def signal_handler(self, signum, frame):
            print("[" + str(self.num) + "]: Received signal " + str(signum)) # + "\n\n" + str(frame))

        def run(self):
            signal.signal(signal.SIGINT, self.signal_handler)
            signal.signal(signal.SIGTERM, self.signal_handler)

            try:
                while True: #not self.tc.shutdown_event.is_set():
                    with self.tc.task_queue.get() as task:
                        if not self.tc.shutdown_event.is_set():
                            msg = "[" + str(self.num) + "]: Do some work (" + str(task)
                            print(msg)
                            do_stuff()
                        else:
                            print("[" + str(self.num) + "]: Ignoring task " + str(task))

            except GracefulShutdownException as e:
                print(str(e))

            print("[" + str(self.num) + "]: Exiting!")

    def make_tasks_gen(num):
        for i in range(num):
            yield {"index": i}

    def make_tasks_list(num):
        return [i for i in make_tasks_gen(num=num)]


    tc = TaskCommunication()

    processes = [Worker(tc=tc, num=i) for i in range(num)]
    print("Created [" + str(num) + "] Workers")

    def shutdown():
        tc.shutdown()

    def signal_handler(signum, frame):
        print("Main process received signal " + str(signum))
        #print("Main process: \n\n" + str(signum) + "\n\n" + str(frame))
        shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    #tc.add_tasks(make_tasks_list(num=30))
    for _ in range(5):
        tc.add_tasks(make_tasks_list(num=100))

    for p in processes:
        p.start()
    print("Started Workers")

    tc.add_tasks(make_tasks_gen(num=500))


    if False:
        time.sleep(20)
    else:
        tc.wait_to_finish()

    print("Sending shutdown signal...")
    tc.shutdown()

    for p in processes:
        p.join()

    print("All Done")




def interruptible_queue_object_test():
    def InterruptibleQueue(maxsize=0):
        import multiprocessing as mp
        import multiprocessing.queues

        class InterruptibleQueueImpl(multiprocessing.queues.JoinableQueue):

            def join(self, timeout=None):
                with self._cond:
                    if not self._unfinished_tasks._semlock._is_zero():
                        return self._cond.wait(timeout=timeout)

            def interruptible_join(self):
                while True:
                    self.join(timeout=1)

        '''Returns a queue object'''
        return InterruptibleQueueImpl(maxsize, ctx=mp.get_context())

    class Worker(mp.Process):
        def __init__(self, queue):
            super(Worker,self).__init__()
            self.queue = queue
            self.daemon = True

        def run(self):
            while True:
                task = self.queue.get(block=True, timeout=None)
                do_stuff()
                self.queue.task_done()
                print("Got task: " + str(task))

    queue = InterruptibleQueue(1000)

    worker = Worker(queue=queue)
    print("Created worker")
    worker.start()

    def add_tasks():
        i = 0
        #while True:
        for i in range(int(500)):
            task = {'i':i}
            i+=1
            print("Adding task: " + str(task))
            queue.put(obj=task, block=True, timeout=0)

    task_thread = threading.Thread(target=add_tasks)
    task_thread.start()

    print("Waiting to join thread...")
    task_thread.join()
    print("Joined thread\nWaiting to join queue...")
    #queue.join(timeout=100)
    queue.interruptible_join()
    print("Joined queue\nExiting?")



def interruptible_queue_keyboard_interrupt():
    def InterruptibleQueue(maxsize=0):
        import multiprocessing as mp
        import multiprocessing.queues

        class InterruptibleQueueImpl(multiprocessing.queues.JoinableQueue):

            def join(self, timeout=None):
                with self._cond:
                    if not self._unfinished_tasks._semlock._is_zero():
                        return self._cond.wait(timeout=timeout)

            def interruptible_join(self):
                while True:
                    self.join(timeout=1)

        '''Returns a queue object'''
        return InterruptibleQueueImpl(maxsize, ctx=mp.get_context())

    class Worker(mp.Process):
        def __init__(self, queue):
            super(Worker,self).__init__()
            self.queue = queue
            self.daemon = True

        def run(self):
            try:
                while True:
                    task = self.queue.get(block=True, timeout=None)
                    do_stuff()
                    self.queue.task_done()
                    print("Got task: " + str(task))
            except KeyboardInterrupt as e:
                print("Worker caught exception: " + str(e))

    queue = InterruptibleQueue(1000)

    worker = Worker(queue=queue)
    print("Created worker")
    worker.start()

    def add_tasks():
        i = 0
        #while True:
        for i in range(int(500)):
            task = {'i':i}
            i+=1
            print("Adding task: " + str(task))
            queue.put(obj=task, block=True, timeout=None)

    task_thread = threading.Thread(target=add_tasks)
    task_thread.start()

    try:
        print("Waiting to join thread...")
        task_thread.join()
        print("Joined thread\nWaiting to join queue...")
        #queue.join(timeout=100)
        queue.interruptible_join()
        print("Joined queue\nExiting?")
    except KeyboardInterrupt as e:
        print("Main process caught exception: " + str(e))



if __name__ == "__main__":
    #run_basic_test(2)
    #run_queue_test(1)
    #run_queue_wrapper_test(5)
    #interruptible_queue_object_test()
    interruptible_queue_keyboard_interrupt()