import multiprocessing as mp
import threading
import time


def processing_stages_test(num):
    import threading
    import queue
    import types
    import signal

    class GracefulShutdownException(BaseException):
        pass

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

    import enum
    @enum.unique
    class RunConditions(enum.Flag):
        NONE         = 0
        #WAIT_FOR_GET = enum.auto()      #current stage should block if necessary to get task from input queue
        WAIT_FOR_PUT = enum.auto()      #prior stage should block if necessary to put task on this stage's input queue
        PROCESS_NEXT = enum.auto()      #keep going
        PROCESS_CURRENT = enum.auto()   #don't interrupt current task
        ALL = WAIT_FOR_PUT | PROCESS_NEXT | PROCESS_CURRENT

    class ShutdownEventInterface(object):

        def __init__(self, name, run_conditions):
            self.name = name
            self.run_conditions = run_conditions
            self.wait_for_finish_event = mp.Event()

        def set_global_events(self, global_events):
            self.map = {RunConditions.WAIT_FOR_PUT: global_events.wait_for_put_event,
                        #RunConditions.WAIT_FOR_GET: global_events.wait_for_get_event,
                        RunConditions.PROCESS_NEXT: global_events.process_next_event,
                        RunConditions.PROCESS_CURRENT: global_events.process_next_event}

        def wait_for_put(self):
            return self.evaluate_condition(condition=RunConditions.WAIT_FOR_PUT)

        def wait_for_get(self):
            #return self.evaluate_condition(condition=RunConditions.WAIT_FOR_GET)
            return not self.waiting_for_finish()

        def process_next(self):
            return self.evaluate_condition(condition=RunConditions.PROCESS_NEXT)

        def process_current(self):
            return self.evaluate_condition(condition=RunConditions.PROCESS_CURRENT)

        def evaluate_condition(self, condition):
            if not self.waiting_for_finish():
                return True
            else:
                res = condition & self.run_conditions
                if res:
                    return True
                else:
                    event = self.map[condition]
                    if not event.is_set():
                        return True
            return False

        def wait_for_finish(self):
            self.wait_for_finish_event.set()

        def waiting_for_finish(self):
            return self.wait_for_finish_event.is_set()

    class InterruptibleQueueWrapper(object):

        def __init__(self, queue, shutdown_events, sleep_time=1, name="queue"):
            self.queue = queue
            self.events = shutdown_events
            self.sleep_time = sleep_time
            self.name = name

        def put(self, task):
            is_full = False
            while True:
                try:
                    self.queue.put(task, block=True, timeout=self.sleep_time)
                except queue.Full as e:
                    if not is_full:
                        #print("[" + str(self.name) + "] is full!, unable to add task!")
                        pass
                    is_full = True
                    if not self.events.wait_for_put():
                        pass
                        #print("Not waiting to put task to [" + str(self.name) + "]")
                        #raise GracefulShutdownException("Not waiting to put task to [" + str(self.name) + "]") #This prevents it from clearing out the input queueu
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
                                #print("No task in [" + str(self.name) + "]!")
                                pass
                            is_empty = True
                            if not self.events.wait_for_get():
                                raise GracefulShutdownException("Done getting tasks from [" + str(self.name) + "]")
                        else:
                            print("Got task " + str(task) + " from [" + str(self.name) + "]!")
                            return task

                def __exit__(myself, exc_type, exc_val, exc_tb):
                    # if exc_type is not None and issubclass(exc_type, NextGetter.DoneException):
                    #    return True
                    self.queue.task_done()

            return NextGetter()

        def join(self):
            return self.queue.join()

    class TaskProcessingStage(object):

        def __init__(self, name, use_mp=False, run_conditions=RunConditions.NONE):
            self.name = name
            self.use_mp = use_mp
            self.events = ShutdownEventInterface(name=name,
                                                 run_conditions=run_conditions)  # defines whether- and how much- stage processes after receiving shutdown request

            self.input_queue = None
            self.output_queue = None
            self.prev_stage = None
            self.next_stage = None
            self.task_exec = mp.Process(target=self.run, daemon=False) if self.use_mp else threading.Thread(
                target=self.run, daemon=False)

            self.num_processed = mp.Value('i', 0)
            self.num_ignored = mp.Value('i', 0)

        def set_global_events(self, global_events):
            self.events.set_global_events(global_events=global_events)

        def set_input_stage(self, stage):
            if stage is not None:
                stage.set_next_stage(self)
                # stage.output_queue = self.input_queue

        def set_next_stage(self, stage):
            self.next_stage = stage
            self.output_queue = stage.input_queue

        def start(self):
            print(self.name + ": Starting " + ("Process" if self.use_mp else "Thread"))
            self.task_exec.start()

        def run(self):
            if self.use_mp:
                signal.signal(signal.SIGINT, self.signal_handler)
                signal.signal(signal.SIGTERM, self.signal_handler)
            try:
                while True:
                    with self.input_queue.get() as task:
                        self.handle_task(task=task)

            except GracefulShutdownException as e:
                print(str(e))

            self.done()

        def signal_handler(self, signum, frame):
            # print("[" + str(self.name) + "]: Received signal " + str(signum))
            pass

        def handle_task(self, task):
            if self.events.process_next():
                self.process_task(task=task)
                self.num_processed.value += 1
            else:
                self.ignore_task(task=task)
                self.num_ignored.value += 1

        def process_task(self, task):
            print(self.name + ": You must implement 'process_task' in your processing stage!")
            raise NotImplementedError("You must implement 'process_task'!")

        def ignore_task(self, task):
            if self.num_ignored.value == 0:
                print(self.name + ": Ignoring this and all following tasks " + str(task))

        def join_input(self):
            self.events.wait_for_finish()
            print(self.name + ": Waiting to join queue [" + str(self.input_queue.name) + "]")
            self.input_queue.join()
            print(self.name + ": Emptied input queue [" + str(self.input_queue.name) + "]")

        def join_exec(self):
            self.task_exec.join()
            print(self.name + ": Joined exec " + ('Thread' if not self.use_mp else 'Process') + ', processed: ' +
                  str(self.num_processed.value) + ', ignored: ' + str(self.num_ignored.value))

        def wait_for_finish(self, source="Unknown"):
            start_t = time.time()
            print(self.name + ": Waiting for finish [" + str(source) + "]")
            self.join_input()
            self.join_exec()

            print(self.name + ": took " + str(time.time() - start_t) + "s to join input and exec [" + str(source) + "]")

            if self.next_stage is not None:
                self.next_stage.wait_for_finish(source=(source + "=>" + self.name))

        # Not needed?
        def done(self):
            print(self.name + ": Finished processing")

    # Takes in lists (or generators) of tasks and transfers them, in order, to the task queue
    class TaskInput(TaskProcessingStage):

        def __init__(self):
            super(TaskInput, self).__init__(name="Task Input", use_mp=False, run_conditions=RunConditions.NONE)
            self.input_queue = InterruptibleQueueWrapper(queue=queue.Queue(10), shutdown_events=self.events,
                                                         sleep_time=1,
                                                         name="Task Input Queue")

        def handle_task(self, task):
            task_it = task  # if isinstance(task, types.GeneratorType) else task if isinstance(task, list) else None
            for t in task_it:
                super(TaskInput, self).handle_task(task=t)

        def process_task(self, task):
            # NOTE: This can end up blocking indefinitely, because the event that would make it return early is only set.... I don't understand
            self.output_queue.put(task)

        def add_tasks(self, tasks):
            self.input_queue.put(task=tasks)

    class ResultRecorder(TaskProcessingStage):

        def __init__(self):
            super(ResultRecorder, self).__init__(name="Result Recorder", use_mp=False, run_conditions=RunConditions.ALL)
            self.input_queue = InterruptibleQueueWrapper(queue=mp.JoinableQueue(maxsize=10),
                                                         shutdown_events=self.events, sleep_time=1,
                                                         name="Result Queue")

        def run(self):
            # with filewriter
            super(ResultRecorder, self).run()
            #print(self.name + ": Finished processing all current results")

        def process_task(self, task):
            print(self.name + ": Processed result " + str(task))

    """
    When each component should stop and how it should be told:
        Task source: task_input_queue
            wait_for_finish: sets task_input_queue's shutdown signal and calls join on it; (also join the input thread)
            once joined, calls wait_for_finish on workers

        Worker pool: task_queue: 
            wait_for_finish: sets task_queue's shutdown signal and calls join on it, then on workers themselves



    """

    # class Job(object):
    #     def __init__(self, i=1e6, step=1e4):
    #
    #         self.target = i
    #         self.step = step
    #

    def interruptible_work(events, i=1e6, denom=1000):
        s = 0
        j = 0
        while j < i and events.process_current():
            while j < i and not j % denom == 0:
                s += j
                j += 1
            print("value of j when checking if current " + str(j) + ": state of flag= " + str(events.process_current()))
            s += j
            j += 1
        if j < i:
            #raise GracefulShutdownException("Task interrupted!")   #This doesn't empty out the input_queue, might be ok for instant shutdown if have interruptible joings
            print("Task interrupted!")
        return s

    def busy_work(i=1e6):
        s = 0
        for i in range(int(i)):
            s += i
        return s

    class Worker(TaskProcessingStage):

        def __init__(self, task_queue, num):
            super(Worker, self).__init__(name="Worker_" + str(num), use_mp=True, run_conditions=RunConditions.NONE)
            self.input_queue = task_queue

        def process_task(self, task):
            msg = "[" + str(self.name) + "]: Do some work (" + str(task) + ")"
            print(msg)
            # busy_work(i=1e8)
            interruptible_work(events=self.events, i=1e8, denom=1e6)
            task["result"] = 'Done'
            task["worker"] = self.name
            self.output_queue.put(task)

    class WorkerPool(TaskProcessingStage):

        def __init__(self, num_workers=1):
            super(WorkerPool, self).__init__(name="Worker Pool", use_mp=False, run_conditions=RunConditions.NONE)
            self.input_queue = InterruptibleQueueWrapper(queue=mp.JoinableQueue(maxsize=num_workers+1), shutdown_events=self.events,
                                                         sleep_time=1,
                                                         name="Task Queue")
            self.workers = [Worker(task_queue=self.input_queue, num=i) for i in range(num_workers)]
            for w in self.workers:
                w.events = self.events

        def start(self):
            print(self.name + ": Starting workers")
            for w in self.workers:
                w.start()

        def set_next_stage(self, stage):
            super(WorkerPool, self).set_next_stage(stage=stage)
            for w in self.workers:
                w.set_next_stage(stage=stage)

        def join_exec(self):
            print(self.name + ": Joining workers")
            for w in self.workers:
                w.join_exec()
            print(self.name + ": Joined all workers")

    class GlobalShutdownState(object):

        def __init__(self):
            self.wait_for_put_event = mp.Event()
            #self.wait_for_get_event = mp.Event()
            self.process_next_event = mp.Event()
            self.process_current_event = mp.Event()

            signal.signal(signal.SIGINT, self.signal_handler)
            signal.signal(signal.SIGTERM, self.signal_handler)
            pass

        def signal_handler(self, signum, frame):
            print("Main process received signal " + str(signum))
            #cmd = RunConditions.PROCESS_CURRENT
            self.shutdown()

        def shutdown(self, conditions=RunConditions):
            self.wait_for_put_event.set()
            #self.wait_for_get_event.set()
            self.process_next_event.set()
            self.process_current_event.set()

    class TaskProcessingPipeline(object):

        def __init__(self, num_workers):
            self.task_input = TaskInput()
            self.workers = WorkerPool(num_workers=num_workers)
            self.result_recorder = ResultRecorder()
            self.global_shutdown_state = GlobalShutdownState()

            self.stages = [self.task_input, self.workers, self.result_recorder]
            for s in self.stages:
                s.set_global_events(global_events = self.global_shutdown_state)

            for ind in range(1, len(self.stages)):
                self.stages[ind].set_input_stage(self.stages[ind - 1])


        def start(self):
            for stage in self.stages:
                stage.start()

        def add_tasks(self, tasks):
            self.task_input.add_tasks(tasks=tasks)

        def wait_for_finish(self, source="Unknown"):
            self.stages[0].wait_for_finish(source=(str(source) + "=>tpp"))

        def shutdown(self, source="Unknown"):
            # for s in self.stages:
            #    s.shutdown(str(source) + "=>tpp")
            # self.stages[0].shutdown(source=(str(source) + "=>tpp"))
            pass

    def make_tasks_gen(num):
        for i in range(num):
            yield {"index": i}

    def make_tasks_list(num):
        return [i for i in make_tasks_gen(num=num)]

    tpp = TaskProcessingPipeline(num_workers=num)
    tpp.start()


    # tc.add_tasks(make_tasks_list(num=30))
    # for _ in range(5):
    #    tpp.add_tasks(make_tasks_list(num=100))

    # tpp.add_tasks(make_tasks_gen(num=500))
    tpp.add_tasks(make_tasks_gen(num=50))

    tpp.wait_for_finish(source="client")

    print("All Done")



if __name__ == "__main__":
    processing_stages_test(num=1)