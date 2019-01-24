nodeName = "messe_controller"
boxId = 0
placeID = 1
tableID = 3
rate = 1

controllerBaseTopic = "/kollrobot_controller/"
goPositionTopic = controllerBaseTopic + "/GoPositionAction/goal"
pickBoxTopic = controllerBaseTopic + "/PickBoxAction/goal"
placeBoxTopic = controllerBaseTopic + "/PlaceBoxAction/goal"
checkPositionService = controllerBaseTopic + "CheckPosition"