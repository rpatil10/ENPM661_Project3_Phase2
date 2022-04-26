# Class for node information
class Node:
    def __init__(self, node_data, parent_data, total_cost, base_cost, intermediate_nodes):
        self.data = node_data
        self.parent = parent_data
        self.sub_nodes = intermediate_nodes
        self.weight = total_cost
        self.base_weight = base_cost

    # Function to override default comparison function to consider only node-weights
    def __eq__(self, other):
        return self.weight == other.weight

    # Function to compare nodes only based on node-weights
    def __lt__(self, other):
        return self.weight < other.weight

    # Function to compare nodes only based on node-weights
    def __gt__(self, other):
        return self.weight > other.weight

    # Function to get a list containing coordinates and orientation of the node
    def get_node_data(self):
        return self.data

    # Function to get parent node
    def get_parent_node(self):
        return self.parent

    # Function to get a list of all the nodes between current node and its parent
    def get_sub_nodes(self):
        return self.sub_nodes

    # Function to get final cost of the node (cost-to-come + cost-to-goal)
    def get_weight(self):
        return self.weight

    # Function to get cost-to-come for the node
    def get_base_weight(self):
        return self.base_weight

    # Function to set node weight
    def set_weight(self, node_weight):
        self.weight = node_weight

    # Function to set base weight
    def set_base_weight(self, base_cost):
        self.base_weight = base_cost
