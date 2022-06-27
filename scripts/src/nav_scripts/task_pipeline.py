import multiprocessing as mp
import threading
import time


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
        self.events = global_events

    def wait_for_put(self):
        return self.evaluate_condition(condition=RunConditions.WAIT_FOR_PUT)

    def wait_for_get(self):
        #return self.evaluate_condition(condition=RunConditions.WAIT_FOR_GET)
        return not self.waiting_for_finish()

    def process_next(self):
        return self.evaluate_condition(condition=RunConditions.PROCESS_NEXT)

    def process_current(self):
        return self.evaluate_condition(condition=RunConditions.PROCESS_CURRENT, must_be_waiting=False)
        #return

    def evaluate_condition(self, condition, must_be_waiting=True):
        if must_be_waiting and not self.waiting_for_finish():
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
    #class Exception(BaseException): pass
    #class NoMoreTasksException(Exception): pass

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
                        self.queue.task_done()
                        return task

            def __exit__(myself, exc_type, exc_val, exc_tb):
                #self.queue.task_done() #This requires waiting for current task to complete before previous stage can shutdown and notify this stage to wait_for_finish
                pass

        return NextGetter()

    def join(self):
        return self.queue.join()

    def close(self):
        print("Shutting down [" + str(self.name) + "]")
        self.queue.join()
        try:
            self.queue.close()
            self.queue.join_thread()
        except AttributeError as e: #TypeError?
            pass

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
        #self.task_exec = mp.Process(target=self.run, daemon=False) if self.use_mp else threading.Thread(target=self.run, daemon=False)
        self.task_exec = (mp.Process if self.use_mp else threading.Thread)(target=self.run, daemon=False, name=name)

        self.num_processed = mp.Value('i', 0)
        self.num_ignored = mp.Value('i', 0)

    def set_global_events(self, global_events):
        self.events.set_global_events(global_events=global_events)
        self.interrupt_monitor = global_events.get_monitor()

    def set_input_stage(self, stage):
        if stage is not None:
            stage.set_next_stage(self)

    def set_next_stage(self, stage):
        self.next_stage = stage
        self.output_queue = stage.input_queue

    def start(self):
        print(self.name + ": Starting " + ("Process" if self.use_mp else "Thread"))
        self.task_exec.start()

    def run(self):
        if self.use_mp:
            threading.current_thread().setName(self.name + "MainThread")
            signal.signal(signal.SIGINT, self.signal_handler)
            signal.signal(signal.SIGTERM, self.signal_handler)
        try:
            while True:
                with self.input_queue.get() as task:
                    self.handle_task(task=task)

        except GracefulShutdownException as e:
            print(str(e))

        #TODO: Maybe rename this 'cleanup' or something and make it 'final'?
        self.done()

    def signal_handler(self, signum, frame):
        # print("[" + str(self.name) + "]: Received signal " + str(signum))
        pass

    def handle_task(self, task):
        if self.events.process_next() and self.events.process_current():
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
        self.input_queue.close()

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
        self.output_queue.put(task)

    def add_tasks(self, tasks):
        self.input_queue.put(task=tasks)

class ResultRecorder(TaskProcessingStage):

    def __init__(self, queue_size=10, sleep_time=1):

        super(ResultRecorder, self).__init__(name="Result Recorder", use_mp=False, run_conditions=RunConditions.ALL)
        self.input_queue = InterruptibleQueueWrapper(queue=mp.JoinableQueue(maxsize=queue_size),
                                                     shutdown_events=self.events, sleep_time=sleep_time,
                                                     name="Result Queue")

    def process_task(self, task):
        raise NotImplementedError("You must override 'process_task'!")

class TaskProcessingException(Exception):
    def __init__(self, msg="", task=None):
        super(TaskProcessingException, self).__init__()
        self.msg = msg
        self.task = task

    def __str__(self):
        return str(self.msg) + ": task=" + str(self.task) if self.task is not None else ""

    def __repr__(self):
        return self.__str__()

class Worker(TaskProcessingStage):

    def __init__(self, num, run_conditions=RunConditions.NONE):
        super(Worker, self).__init__(name="Worker_" + str(num), use_mp=True, run_conditions=run_conditions)

    def process_task(self, task):
        msg = "[" + str(self.name) + "]: Do some work (" + str(task) + ")"
        print(msg)
        try:
            result = self.task_result_func(task)
        except TaskProcessingException as e:
            result = str(e)
        finally:
            task["result"] = result
            task["worker"] = self.name
            self.output_queue.put(task)

    def task_result_func(self, task):
        raise NotImplementedError("You must either implement 'task_result_func' or override 'process_task'!")


class WorkerPool(TaskProcessingStage):

    def __init__(self, workers):
        super(WorkerPool, self).__init__(name="Worker Pool", use_mp=False, run_conditions=RunConditions.NONE)
        self.workers = workers

        self.input_queue = InterruptibleQueueWrapper(queue=mp.JoinableQueue(maxsize=len(workers)+1), shutdown_events=self.events,
                                                     sleep_time=1,
                                                     name="Task Queue")
        for w in self.workers:
            w.events = self.events
            w.input_queue = self.input_queue

    def start(self):
        print(self.name + ": Starting workers")
        for w in self.workers:
            w.start()

    def set_global_events(self, global_events):
        for w in self.workers:
            w.set_global_events(global_events=global_events)


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

        self.map = {RunConditions.WAIT_FOR_PUT: self.wait_for_put_event,
                    # RunConditions.WAIT_FOR_GET: global_events.wait_for_get_event,
                    RunConditions.PROCESS_NEXT: self.process_next_event,
                    RunConditions.PROCESS_CURRENT: self.process_current_event}

        self.sig_int_counter = 0

        sigint_cond = RunConditions.PROCESS_CURRENT
        sigterm_cond = RunConditions.NONE
        self.shutdown_conditions =  [sigint_cond, sigterm_cond]


    def enable_signals(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, signum, frame):
        maybeprint = lambda c: print(c)
        maybeprint("Main process received signal " + str(signum))
        if signum == signal.SIGINT.value:
            self.sig_int_counter+=1
            maybeprint("SIGINT " + str(self.sig_int_counter), )

            if self.sig_int_counter > 1:
                conditions = self.shutdown_conditions[1]
                maybeprint("Received more than 1 Ctrl+C/SIGINT, shutting down now!!")
            else:
                conditions = self.shutdown_conditions[0]
                maybeprint("Program will exit after completing in-progress tasks. Pressing Ctrl+C again will initiate immediate shutdown")
        elif signum == signal.SIGTERM.value:
            conditions = self.shutdown_conditions[1]
            maybeprint("Received SIGTERM, shutting down now!!")

        self.shutdown(conditions=conditions)

    def shutdown(self, conditions=RunConditions.NONE):
        for cond,event in self.map.items():
            #For each run condition that isn't active, activate the associated event
            if not (cond & conditions):
                event.set()

        ##previous method for shutting down
        # self.wait_for_put_event.set()
        # #self.wait_for_get_event.set()
        # self.process_next_event.set()
        # self.process_current_event.set()

    def get_monitor(self):
        class InterruptMonitor(object):
            def __init__(myself):
                pass

            def update(myself):
                if self.process_current_event.is_set():
                    raise TaskProcessingException("Task interrupted!")

        m = InterruptMonitor()
        return m

class TaskProcessingPipeline(object):

    def __init__(self):
        self.global_shutdown_state = GlobalShutdownState()

    def setup_stages(self, num_workers):
        self.task_input = self.get_task_input()
        workers = [self.get_worker(num=i) for i in range(num_workers)]

        self.workers = WorkerPool(workers=workers)
        self.result_recorder = self.get_result_recorder()

        self.stages = [self.task_input, self.workers, self.result_recorder]
        for s in self.stages:
            s.set_global_events(global_events=self.global_shutdown_state)

        for ind in range(1, len(self.stages)):
            self.stages[ind].set_input_stage(self.stages[ind - 1])

    def start(self):
        self.disable_signals()
        for stage in self.stages:
            stage.start()
        self.global_shutdown_state.enable_signals() #Must be done AFTER worker processes have been created

    def add_tasks(self, tasks):
        self.task_input.add_tasks(tasks=tasks)

    def wait_for_finish(self, source="Unknown"):
        self.stages[0].wait_for_finish(source=(str(source) + "=>tpp"))

    def shutdown(self, source="Unknown", conditions=RunConditions.NONE):
        self.global_shutdown_state.shutdown(conditions=conditions)

    def get_task_input(self):
        return TaskInput()

    def get_worker(self, num):
        raise NotImplementedError("You must override 'get_worker'!")

    def get_result_recorder(self):
        raise NotImplementedError("You must override 'get_result_recorder'!")

    def disable_signals(self):
        def do_nothing(signum, frame):
            #print("Ignoring signal " + str(signum))
            pass

        signal.signal(signal.SIGINT, do_nothing)
        signal.signal(signal.SIGTERM, do_nothing)


def interruptible_work(events, i=1e6, denom=1000):
    denom = int(denom)
    s = 0
    j = 0
    while j < i and events.process_current():
        while j < i and not j % denom == 0:
            s += j
            j += 1
        #print("value of j when checking if current " + str(j) + ": state of flag= " + str(events.process_current()))
        s += j
        j += 1
    if j < i:
        #raise GracefulShutdownException("Task interrupted!")   #This doesn't empty out the input_queue, might be ok for instant shutdown if have interruptible joings
        raise InterruptedError("Task interrupted!")
        #print("Task interrupted!")
    return s

def busy_work(i=1e6):
    s = 0
    for i in range(int(i)):
        s += i
    return s

class DemoWorker(Worker):

    def __init__(self, num):
        super(DemoWorker, self).__init__(num=num)

    def task_result_func(self, task):
        i = task.get('size', 1e7)
        return interruptible_work(events=self.events, i=i, denom=1e6)

class DemoResultRecorder(ResultRecorder):

    def __init__(self):
        super(DemoResultRecorder, self).__init__()

    def process_task(self, task):
        print(self.name + ": Processed result " + str(task))

class DemoTaskProcessingPipeline(TaskProcessingPipeline):

    def __init__(self, num_workers=1):
        super(DemoTaskProcessingPipeline, self).__init__()
        super(DemoTaskProcessingPipeline, self).setup_stages(num_workers=num_workers)

    def get_worker(self, num):
        worker = DemoWorker(num=num)
        return worker

    def get_result_recorder(self):
        return DemoResultRecorder()


def processing_stages_test(num=1):
    tpp = DemoTaskProcessingPipeline(num_workers=num)
    tpp.start()
    import random


    def make_tasks_gen(num):
        for i in range(num):
            yield {"index": i, 'size': random.randint(int(1e5), int(1e8))}

    def make_tasks_list(num):
        return [i for i in make_tasks_gen(num=num)]

    tpp.add_tasks(make_tasks_list(num=30))
    for _ in range(5):
       tpp.add_tasks(make_tasks_list(num=15))

    tpp.add_tasks(make_tasks_gen(num=20))

    tpp.wait_for_finish(source="client")

    print("All Done")



if __name__ == "__main__":
    processing_stages_test(num=15)