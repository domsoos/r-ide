<script>
    const vscode = acquireVsCodeApi();
    import { onMount } from 'svelte';
    import ROS from "../../ROSManagers/rosmanager"
    import ROSLIB from "roslib";
    import { DataSet, Network } from "vis-network/standalone";
    //import vis from "vis-network";

    let rosApi;

    let isLoading = false;
    let isConnected = false;

    onMount(async () => {
    try {
        await new ROS();
        rosApi = ROS.getROSApi();
        getROSNodes();
    } catch (err) {
        isConnected = false;
        vscode.postMessage({
            type: 'r-ide.noConnection',
        });
        //console.error(err);
    }
    });

    
    function clearAllData(){

    }

    function getROSNodes(){
        isLoading = true;

        if(rosApi?.isConnected){
            rosApi.getNodes((res) => {
                if (res) {
                    console.log(res);
                    getNodeDetails(res);
                    console.log(res);
                } else {
                    console.log("failed");
                    isConnected = false;
                }
                isLoading = false;
            }, (err)=>{
                console.log(err);
                vscode.postMessage({
                    type: 'r-ide.noConnection',
			    });
                isLoading = false;
                isConnected = false;
            });
        }else{
            isLoading = false;
            isConnected = false;
            clearAllData();
        }
    }

    function getNodeDetails(nodes) {

        // Initialize nodeData and edgeData
        const nodeData = [];
        const edgeData = [];

        // Update the graph
        const updateGraph = () => {
            const data = {
                nodes: new DataSet(nodeData),
                edges: new DataSet(edgeData),
            };

            const container = document.getElementById("mynetwork");
            container.style.height = "600px";

            const options = {};

            const network = new Network(container, data, options);
        }

        // Get details for each node
        const processNode = (node) =>  {
            rosApi.getNodeDetails(node, (res) => {
                const publishing = res.publishing.map(topic => topic.split(' ')[0]);
                const subscribing = res.subscribing.map(topic => topic.split(' ')[0]);

                console.log(`Node: ${node}`);
                console.log(`Publishing: ${publishing}`);
                console.log(`Subscribing: ${subscribing}`);

                const nodeDataItem = {
                    id: node,
                    label: node,
                    topics: {
                        publishing: publishing,
                        subscribing: subscribing
                    },
                };

                // Add node to nodeData
                nodeData.push(nodeDataItem);
                console.log(`Node ${node} processed.`)

                // Create edges to publishing nodes
                for (let topic of subscribing) {
                    const publishingNodes = nodeData.filter(n => n.topics.publishing.includes(topic));
                    console.log(`Publishing nodes for topic ${topic}: ${publishingNodes.map(n => n.id)}`);
                    for (let publishingNode of publishingNodes) {
                        edgeData.push({
                            from: publishingNode.id,
                            to: nodeDataItem.id,
                            label: topic
                        });
                        console.log(`Edge from ${publishingNode.id} to ${nodeDataItem.id} with label ${topic} added.`)
                    }
                }

                // Create edges to subscribing nodes
                for (let topic of publishing) {
                    const subscribingNodes = nodeData.filter(n => n.topics.subscribing.includes(topic));
                    console.log(`Subscribing nodes for topic ${topic}: ${subscribingNodes.map(n => n.id)}`);
                    for (let subscribingNode of subscribingNodes) {
                        edgeData.push({
                            from: nodeDataItem.id,
                            to: subscribingNode.id,
                            label: topic
                        });
                        console.log(`Edge from ${nodeDataItem.id} to ${subscribingNode.id} with label ${topic} added.`)
                    }
                }

                updateGraph();

                const nextoNode = nodes.shift();

                console.log(`Next node: ${nextoNode}`)

                // Process next node
                if(nextoNode){
                    processNode(nextoNode);
                }
                else {
                    updateGraph();
                }
            });
        };

        // Process the first node
        const initialNode = nodes.shift();

        console.log(`Initial node: ${initialNode}`)

        // Process next node
        if (initialNode) {
            processNode(initialNode);
        }
    }

</script>


<div id="mynetwork"></div>