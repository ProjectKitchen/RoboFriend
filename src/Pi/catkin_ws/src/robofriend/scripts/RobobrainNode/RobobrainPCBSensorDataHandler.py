from RobobrainStateHandler import *

class RobobrainPCBSensorDataHandler():

    def __init__(self):
        self.__bat_value = None
        self.__bat_percent = None
        self.__inf_left = None
        self.__inf_middle = None
        self.__inf_right = None

        self.__batWasLow = None
        # self.__robostate_obj = robostate

    def process_data(self, data):
        self.__bat_value = round(data.bat_voltage, 2)
        self.__bat_percent = data.bat_percent
        self.__inf_left = round(data.inf_left, 2)
        self.__inf_middle = round(data.inf_middle, 2)
        self.__inf_right = round(data.inf_right, 2)
        # print("[INFO] {} - Received message from Battery/Infrared Node : {}, {}, {}, {}, {}\n"
        #         .format(__class__.__name__, self.__bat_value, self.__bat_percent, self.__inf_left,
        #         self.__inf_middle, self.__inf_right))

        self.__data_evaluate()

    def __data_evaluate(self):
        if self.__bat_value < 12 and self.__bat_value > 11.8:
            self.__batWasLow = True
            print("[INFO] {} - Battery at low level change state to FIND_CHARGING_STATION!".format(__class__.__name__))
            self.__robostate_obj.state = RobobrainStateHandler.robostate["FIND_CHARGING_STATION"]
        elif self.__bat_value < 11.8:
            print("[INFO] {} - Battery reached critical level, going shutdown!".format(__class__.__name__))
            self.__robostate_obj.state = RobobrainStateHandler.robostate["SHUTDOWN"]
        if self.__batWasLow == True and self.__bat_value > 12.15:
            batwWasLow = False
            #print("[INFO] Robofriend is recharged!")
