import rospy
import shelve
import time
from threading import Lock, Timer
from robonomics_liability.msg import Liability, LiabilityExecutionTimestamp
from robonomics_liability.srv import PersistenceContainsLiability, PersistenceContainsLiabilityResponse, PersistenceLiabilityTimestamp, PersistenceLiabilityTimestampResponse
from persistent_queue import PersistentQueue


class LiabilityExecutionsPersistence:
    def __init__(self):
        """
            Robonomics liability persistence node initialisation.
        """
        rospy.init_node('robonomics_liability_persistence')
        self.persistence_update_interval = rospy.get_param('~persistence_update_interval', 0.1)

        self.__liability_executions_lock = Lock()
        self.__liability_timestamps_lock = Lock()

        self.__liability_executions            = shelve.open("robonomics_liability_executions.persistence")
        self.__liability_executions_timestamps = shelve.open("robonomics_liability_executions_timestamps.persistence")
        self.__liability_executions_timestamps_queue = PersistentQueue('robonomics_liability_executions_timestamps.queue')

        rospy.Subscriber('persistence/add', Liability, self.__add_liability)
        rospy.Subscriber('persistence/del', Liability, self.__del_liability)
        rospy.Subscriber("persistence/update_timestamp", LiabilityExecutionTimestamp,
                         self.__update_liability_execution_timestamp_handler)
        rospy.Service("persistence/exists", PersistenceContainsLiability, self.__liability_exists)
        rospy.Service("persistence/get_liability_timestamp", PersistenceLiabilityTimestamp, self.__get_liability_timestamp)

        self.__incoming_liability_topic = rospy.Publisher('incoming', Liability, queue_size=10)
        self.__restore_executions()

    def __update_liability_execution_timestamp_handler(self, msg):
        self.__liability_executions_timestamps_queue.push(msg)

    def __update_liability_execution_timestamp(self, msg):
        rospy.logdebug("update liability %s execution timestamp", msg.address.address)
        if msg.address.address not in self.__liability_executions_timestamps:
            rospy.logwarn("liability %s already unregistered from timestamps persistence",
                          msg.address.address)
            return

        try:
            self.__liability_timestamps_lock.acquire()
            self.__liability_executions_timestamps[msg.address.address] = msg.timestamp
            self.__liability_executions_timestamps.sync()
        finally:
            self.__liability_timestamps_lock.release()
        rospy.logdebug("Persistence liability %s timestamp %s",
                       msg.address.address,
                       self.__liability_executions_timestamps[msg.address.address])

    def __liability_exists(self, msg):
        return PersistenceContainsLiabilityResponse(msg.address in self.__liability_executions)

    def __register_liability_in_timestamp_persistence(self, msg):
        try:
            self.__liability_timestamps_lock.acquire()
            if msg.address.address not in self.__liability_executions_timestamps:
                self.__liability_executions_timestamps[msg.address.address] = rospy.Time.from_sec(0)
            rospy.loginfo("Timestamps persistence contains %s value for liability %s",
                          self.__liability_executions_timestamps[msg.address.address],
                          msg.address.address)
            self.__liability_executions_timestamps.sync()
        finally:
            self.__liability_timestamps_lock.release()

    def __add_liability(self, msg):
        try:
            self.__liability_executions_lock.acquire()
            self.__liability_executions[msg.address.address] = msg
            self.__register_liability_in_timestamp_persistence(msg)
            self.__liability_executions.sync()
            rospy.loginfo("Liability %s added to liabilities executions persistence store", msg.address.address)
        finally:
            self.__liability_executions_lock.release()

    def __del_liability(self, msg):
        try:
            self.__liability_executions_lock.acquire()
            del self.__liability_executions[msg.address.address]
            self.__liability_executions.sync()
            rospy.loginfo("Liability %s deleted from liabilities executions persistence store", msg.address.address)
        except KeyError:
            rospy.logwarn("Liability %s not found in liabilities executions persistence store", msg.address.address)
        finally:
            self.__liability_executions_lock.release()

        try:
            self.__liability_timestamps_lock.acquire()
            del self.__liability_executions_timestamps[msg.address.address]
            self.__liability_executions_timestamps.sync()
            rospy.loginfo("Liability %s deleted from liabilities timestamps persistence store", msg.address.address)
        except KeyError:
            rospy.logwarn("Liability %s not found in liabilities timestamps persistence store", msg.address.address)
        finally:
            self.__liability_timestamps_lock.release()

    def __restore_executions(self):
        try:
            self.__liability_executions_lock.acquire()
            executions = list(self.__liability_executions.values())
        finally:
            self.__liability_executions_lock.release()

        time.sleep(5)
        for liability in executions:
            self.__incoming_liability_topic.publish(liability)
            rospy.logwarn("Liability %s received from liabilities executions persistence store",
                          liability.address.address)

    def __get_liability_timestamp(self, msg):
        timestamp = rospy.Time.from_sec(0)
        liability_address = msg.address.address
        queue_entry = self.__liability_executions_timestamps_queue.peek()
        while queue_entry is not None:
            time.sleep(0.1)
            queue_entry = self.__liability_executions_timestamps_queue.peek()
        try:
            self.__liability_timestamps_lock.acquire()
            timestamp = self.__liability_executions_timestamps[liability_address]
        except KeyError as e:
            rospy.logwarn("Unable to get known execution timestamp for liability %s", liability_address)
        finally:
            self.__liability_timestamps_lock.release()
        return PersistenceLiabilityTimestampResponse(timestamp)

    def spin(self):
        def update_liability_timestamp_queue_handler():
            entry = self.__liability_executions_timestamps_queue.peek()
            if entry is not None:
                self.__update_liability_execution_timestamp(entry)
                self.__liability_executions_timestamps_queue.pop()
            Timer(self.persistence_update_interval, update_liability_timestamp_queue_handler).start()

        update_liability_timestamp_queue_handler()
        rospy.spin()
