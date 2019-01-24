nodeName = "messe_controller"
boxId = 0
placeID = 1
tableID = 3
rate = 1
posReachedRetry = 10

controllerBaseTopic = "kollrobot_controller/"
goPositionTopic = controllerBaseTopic + "GoPositionAction"
pickBoxTopic = controllerBaseTopic + "PickBoxAction"
placeBoxTopic = controllerBaseTopic + "PlaceBoxAction"
checkPositionService = controllerBaseTopic + "CheckPosition"
checkTrajectoryService = controllerBaseTopic + "IsTrajectoryExecuting"