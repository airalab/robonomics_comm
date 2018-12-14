import rospy, shelve, time
from threading import Lock
from robonomics_liability.msg import Liability
from robonomics_liability.srv import PersistenceContainsLiability, PersistenceContainsLiabilityResponse


class LiabilityExecutionsPersistence:
    def __init__(self):
        """
            Robonomics liability persistence node initialisation.
        """
        rospy.init_node('robonomics_liability_persistence')

        self.__liability_executions_lock = Lock()
        self.__liability_executions = shelve.open("robonomics_liability_executions.persistence")

        self.__incoming_liability_topic = rospy.Publisher('incoming', Liability, queue_size=10)

        self.__restart_executions()

        rospy.Subscriber('persistence/add', Liability, self.__add_liability)
        rospy.Subscriber('persistence/del', Liability, self.__del_liability)

        rospy.Service("persistence/exists", PersistenceContainsLiability, self.__liability_exists)

    def __liability_exists(self, msg):
        return PersistenceContainsLiabilityResponse(msg.address in self.__liability_executions)

    def __add_liability(self, msg):
        try:
            self.__liability_executions_lock.acquire()
            self.__liability_executions[msg.address.address] = msg
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

    def __restart_executions(self):
        try:
            self.__liability_executions_lock.acquire()
            executions = list(self.__liability_executions.values())
        finally:
            self.__liability_executions_lock.release()

        time.sleep(5)
        for liability in executions:
            self.__incoming_liability_topic.publish(liability)
            rospy.logwarn("Liability %s received from liabilities executions persistence store", liability.address.address)

    def spin(self):
        rospy.spin()
