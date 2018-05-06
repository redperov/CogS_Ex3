class Plan:
    # Represents a node in the plan
    class Node:
        def __init__(self, nodeName, actionName, preConds, termConds):
            self.nodeName = nodeName
            self.action = actionName
            self.preConds = preConds
            self.termConds = termConds
            self.hierarchicalChildren = []
            self.sequentialFollowers = []

    # Initiate an empty list of nodes
    def __init__(self):
        self.nodes = []
        self.rank_list = []

    # Adds a node to the plan
    def addNode(self, nodeName, actionName, preConds, termConds, inConditions=[]):
        if self.isNodeInPlan(nodeName):
            print nodeName
            raise Exception('Node name must be unique' )
        self.nodes.append(self.Node(nodeName, actionName, preConds, termConds))


    def isNodeInPlan(self, nodeName):
        if nodeName in [node.nodeName for node in self.nodes]:
            return True
        return False


    # Gets a node by name
    def getNode(self, nodeName):
        index = [node.nodeName for node in self.nodes].index(nodeName)
        return self.nodes[index]

    # Adds an hierarchical edge
    def addHierarchicalEdge(self, parentNodeName, childNodeName):
        # Checks if the nodes we are connecting were set beforehand
        if not self.isNodeInPlan(parentNodeName):
            raise Exception(str.format('No such node {0}', parentNodeName))
        if not self.isNodeInPlan(childNodeName):
            raise Exception (str.format('No such node {0}', childNodeName))
        parentNode = self.getNode(parentNodeName)
        childNode = self.getNode(childNodeName)
        parentNode.hierarchicalChildren.append(childNode)
        
    # Adds a sequential edge
    def addSequentialEdge(self, parentNodeName, childNodeName):
        # Checks if the nodes we are connecting were set beforehand
        if not self.isNodeInPlan(parentNodeName):
            raise Exception(str.format('No such node {0}', parentNodeName))
        if not self.isNodeInPlan(childNodeName):
            raise Exception (str.format('No such node {0}', childNodeName))
        parentNode = self.getNode(parentNodeName)
        childNode = self.getNode(childNodeName)
        parentNode.sequentialFollowers.append(childNode)


    def getNodeRank(self, nodeName):
        for i in range(len(self.rank_list)):
            if nodeName in self.rank_list[i]:
                return i

        return -1

    @staticmethod
    def str_list_for_dot(mylist):
        new_str = mylist.__str__().strip("[,]")
        new_str = new_str.replace("\'", "")
        return new_str

    def writeDotNodes(self, node, curr_rank, f):
        if curr_rank == len(self.rank_list):
            self.rank_list.append([node.nodeName])
        else:
            self.rank_list[curr_rank].append(node.nodeName)
        # name_children = [child.nodeName for child in node.hierarchicalChildren]
        # f.write("{} -> {}{}{} [style=dashed]\n".format(node.nodeName, "{", Plan.str_list_for_dot(name_children), "}"))
        for child in node.hierarchicalChildren:
            f.write("{} -> {}{}{} [style=dashed]\n".format(node.nodeName, "{", child.nodeName, "}"))
            self.writeDotNodes(child, curr_rank + 1, f)

        # name_followers = [follow.nodeName for follow in node.sequentialFollowers]
        # f.write("{} -> {}{}{}\n".format(node.nodeName, "{", Plan.str_list_for_dot(name_followers), "}"))
        for follower in node.sequentialFollowers:
            f.write("{} -> {}{}{}\n".format(node.nodeName, "{", follower.nodeName, "}"))
            is_ranked = self.getNodeRank(follower.nodeName)
            if is_ranked != -1:
                continue
            self.writeDotNodes(follower, curr_rank, f)

    def toDotFile(self, path):
        f = open(path, "w")
        # f.write("digraph G {\n rankdir=LR;\n")
        # f.write("digraph G {\n splines=line;\n")
        f.write("digraph G {\n")
        self.writeDotNodes(self.nodes[0], 0, f)

        for rank in self.rank_list:
            f.write("{ rank=same; ")
            f.write(Plan.str_list_for_dot(rank))
            f.write("}\n")

        # ending the graph
        f.write("}")
        f.close()

    def printPlan(self):
        for node in self.nodes:
            print node.nodeName
            print "action name:", node.action
            print "precond:", node.preConds
            print "termCond:", node.termConds
            print "hierarchicalChildren: ", [n.nodeName for n in node.hierarchicalChildren]
            print "sequentialFollowers: ", [n.nodeName for n in node.sequentialFollowers]
            print "--------------------------------------------------------------------------------"
