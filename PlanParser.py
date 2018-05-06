import xml.etree.ElementTree as ET
from Plan import Plan

class PlanParser:
    # XML parser for creating a plan
    def __init__(self, fileName):
        self.fileName = fileName
        self.plan = Plan()

    def parseNode(self, nodeData):
        # Gets the node's data from the node element
        nodeName = nodeData.attrib['nodeName']
        if nodeData.find('actionName') is not None:
            action = nodeData.find('actionName').text
        else:
            action = None

        preConds = dict()
        for preCond in nodeData.findall('preCond'):
            variables = preCond.attrib["key"].split("?")[1:]
            groundedVariables = None
            for variable in variables:
                if variable in preCond.attrib:
                    if not groundedVariables:
                        groundedVariables = dict()
                    groundedVariables[variable] = preCond.attrib[variable]
            preConds[preCond.attrib["key"]] = (preCond.attrib["value"], groundedVariables)

        termConds = dict()
        for termCond in nodeData.findall('termCond'):
            variables = termCond.attrib["key"].split("?")[1:]
            groundedVariables = None
            for variable in variables:
                if variable in termCond.attrib:
                    if not groundedVariables:
                        groundedVariables = dict()
                    groundedVariables[variable] = termCond.attrib[variable]

            if "op" in termCond.attrib:
                termConds[termCond.attrib["key"]] = (termCond.attrib["value"], termCond.attrib["op"], groundedVariables)
            else:
                termConds[termCond.attrib["key"]] = (termCond.attrib["value"], None, groundedVariables)


        self.plan.addNode(nodeName = nodeName, actionName = action, preConds = preConds, termConds = termConds)

    def parseHierarchicalEdge(self, hierarchicalEdgeData):
        # Gets the edge's attributes
        fromNode = hierarchicalEdgeData.get('from')
        toNode = hierarchicalEdgeData.get('to')
        # Adds the edge to the plan
        self.plan.addHierarchicalEdge(fromNode, toNode)

    def parseSequentialEdge(self, sequentialEdgeData):
        # Gets the edge's attributes
        fromNode = sequentialEdgeData.get('from')
        toNode = sequentialEdgeData.get('to')
        # Adds the edge to the plan
        self.plan.addSequentialEdge(fromNode, toNode)

    def getPlan(self):
        tree = ET.parse(self.fileName)
        root = tree.getroot()

        # Adds all nodes to the plan
        nodes = root.find('nodes')
        for node in nodes:
            self.parseNode(node)

        # Adds all edges to the plan
        heirarchicalEdges = root.find('heirarchicalEdges')
        if heirarchicalEdges is not None:
            for heirarchicalEdge in heirarchicalEdges:
                self.parseHierarchicalEdge(heirarchicalEdge)
        sequentialEdges = root.find('sequentialEdges')
        if sequentialEdges is not None:
            for sequentialEdge in sequentialEdges:
                self.parseSequentialEdge(sequentialEdge)
        return self.plan




