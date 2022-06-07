import multiprocessing as mp
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
                    self.queue.put(task, block=False) #, block=True, timeout=self.sleep_time)
                except queue.Full as e:
                    if not is_full:
                        print("[" + str(self.name) + "] is full!, unable to add task!")
                    is_full = True
                    time.sleep(self.sleep_time)
                else:
                    print("Added task " + str(task) + " to [" + str(self.name) + "]")
                    break

        def get(self):

            class NextGetter(object):

                #class DoneException(BaseException): pass
                #
                # def __init__(self, queue, name):
                #     self.queue = queue
                #     self.name = name

                def __enter__(myself):
                    is_empty = False

                    while not self.shutdown_event.is_set():
                        try:
                            task = self.queue.get(block=False)
                        except queue.Empty as e:
                            if not is_empty:
                                print("No task in [" + str(self.name) + "]!")
                            is_empty = True
                            time.sleep(1)
                        else:
                            print("Got task " + str(task) + " from [" + str(self.name) + "]!")
                            return task

                    raise GracefulShutdownException()

                def __exit__(myself, exc_type, exc_val, exc_tb):
                    #if exc_type is not None and issubclass(exc_type, NextGetter.DoneException):
                    #    return True
                    self.queue.task_done()

            return NextGetter()

    class TaskCommunication(object):

        #Takes in lists (or generators) of tasks and transfers them, in order, to the task queue
        class TaskQueuer(object):

            def __init__(self, task_queue, shutdown_event):
                self.task_queue = InterruptibleQueueWrapper(queue=task_queue, shutdown_event=shutdown_event, sleep_time=1, name="Task Queue")
                self.thread = threading.Thread(target=self.run)
                self.task_input_queue = InterruptibleQueueWrapper(queue=queue.Queue(10), shutdown_event=shutdown_event, sleep_time=1, name="Task Input Queue")
                self.shutdown_event = shutdown_event
                self.thread.start()

            def run(self):

                try:
                    while not self.shutdown_event.is_set():
                        with self.task_input_queue.get() as task_obj:
                            task_it = task_obj() if isinstance(task_obj, types.GeneratorType) else task_obj if isinstance(task_obj, list) else None

                            for t in task_it:
                                self.task_queue.put(t)
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
            self.task_queue = mp.JoinableQueue(maxsize=10)
            self.result_queue = mp.JoinableQueue(maxsize=10)
            self.shutdown_event = mp.Event()
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
                    msg = "[" + str(self.num) + "]: Do some work (" + str(task)
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

    tc.add_tasks(make_tasks_list(num=30))

    for p in processes:
        p.start()
    print("Started Workers")

    for _ in range(3):
        tc.add_tasks(make_tasks_list(num=10))
    tc.add_tasks(make_tasks_list(num=50))


    if False:
        time.sleep(20)
    else:
        tc.wait_to_finish()

    print("Sending shutdown signal...")
    tc.shutdown()

    for p in processes:
        p.join()

    print("All Done")




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

if __name__ == "__main__":
    #run_basic_test(2)
    #run_queue_test(1)
    run_queue_wrapper_test(5)