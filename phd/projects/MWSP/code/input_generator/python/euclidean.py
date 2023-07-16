#!/usr/bin/python
# from time import localtime
import random
import networkx as nx
import math

# activities = {8: 'Sleeping',
#               9: 'Commuting',
#               10: 'Working',
#               18: 'Commuting',
#               20: 'Eating',
#               22: 'Resting' }

# time_now = localtime()
# hour = time_now.tm_hour

# for activity_time in sorted(activities.keys()):
#     if hour < activity_time:
#         print activities[activity_time]
#         break
# else:
#     print 'Unknown, AFK or sleeping!'

tmpdir = "/tmp/"
outputFileName = "input.dat"
qtInst = 50
for n in [20, 30, 40, 50]:
    print 'n: ' + str(n)
    for d in [5, 10, 20, 25]:
        print 'd: ' + str(d)
        for l in range(qtInst):
            print 'l: ' + str(l)
            # G = nx.hypercube_graph(d)
            # print 'criou grafo'

            numOfNodes = math.pow(2,d)
            if (numOfNodes >= n):            
            # if (G.number_of_nodes() >= n):
                # print 'dentro de if'
                
                # no_graph = True
                # rand_numbers = random.sample(range(G.number_of_nodes()), n)

                nodesList = []
                for g in range(n):                    
                    flag = False
                    while(not flag):
                        intNode = random.randint(0, numOfNodes - 1)
                        if intNode not in nodesList:
                            nodesList.append(intNode)
                            flag = True                
                            
                    
                # while(no_graph):
                    # print 'd: ' + str(d) + ', l: ' + str(l) + ', G.num_nodes: ' + str(G.number_of_nodes()) + ', n: ' + str(n)
                    # rand_numbers = random.sample(range(G.number_of_nodes()), (G.number_of_nodes() - n))
                    # rand_numbers = random.sample(range(G.number_of_nodes()), n)
                    # nodes2Del = []
                    # for i in rand_numbers:
                    #     nodes2Del.append(G.nodes()[i])
                    
                    # G.remove_nodes_from(nodes2Del)
                        
                    # if nx.is_connected(G):
                    #     no_graph = False
                    # else:
                    #     G = nx.hypercube_graph(d)
                    # no_graph = False
                        
                ind1 = -1
                nodes = []

                # print 'depois de rand_numbers'

                # print str((G.number_of_nodes() - n))
                file = open(tmpdir + str(l+1) + "-" + str(d) + "-" + str(n) + "-euclidean-dim-" + outputFileName, "w")
                # print 'depois de open'
                # file.write(str(n) + " " + str(G.size()) + "\n")
                file.write(str(n) + " " + str((n * (n-1))/2) + "\n")

                # print 'depois de write'
                # for i in range(G.number_of_nodes()):
                #     if (len(filter (lambda x : x == i, rand_numbers)) == 0):
                #         print 'encontrou no ' + str(i)
                #         ind1 = ind1 + 1
                #         nodes[ind1] = i

                for i in range(n):
                    for j in range(i+1, n):
                        sum = 0
                        for k in range(d):
                            expr = "{:" + str(d) + "b}"
                            binaryNodei = [1 if x=='1' else 0 for x in expr.format(nodesList[i])]
                            binaryNodej = [1 if x=='1' else 0 for x in expr.format(nodesList[j])]
                            sum = sum + math.pow((binaryNodei[k] - binaryNodej[k]), 2)
                            # sum = sum + math.pow((G.nodes()[rand_numbers[i]][k] - G.nodes()[rand_numbers[j]][k]), 2)
                            
                        cost = int(math.sqrt(sum) * 100)
                        # print 'depois de custo'
                        file.write(str(i) + " " + str(j) + " " + str(cost) + "\n")
                        # print 'depois de write custo para ' + str(i) + ' ' + str(j) 
                        # file.write(str(nodes[i]) + " " + str(nodes[j]) + " " + str(cost))

                file.close()
                # print 'depois de close'
