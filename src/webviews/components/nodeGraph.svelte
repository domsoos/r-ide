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

    let nodeDetails = [];

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

    function getNodeDetails() {
        rosApi.getNodes((nodes) => {
            console.log("Active Nodes:", nodes);
            
            const nodeData = [];
            const edgeData = [];

            // create a node for each active node in the ROS network
            for (const node of nodes) {
            nodeData.push({ id: node, label: node });
            }

            // get the topics used by the active nodes
            rosApi.getTopics((topics) => {
            console.log("Topics:", topics.topics);
            for (const topic of topics.topics) {
                console.log(`Checking topic ${topic}`);
                // Loop over each pair of nodes and check if their topics overlap
                for (let i = 0; i < nodes.length; i++) {
                for (let j = i + 1; j < nodes.length; j++) {
                    // Check if the nodes share a topic
                    if (nodes[i].topic == nodes[j].topic) {
                    console.log(`Adding edge between ${nodes[i]} and ${nodes[j]} for topic ${topic}`);
                    // If they do, create an edge between them
                    edgeData.push({ from: nodes[i], to: nodes[j], label: topic });
                    } 
                    else {
                    console.log(`No edge added between ${nodes[i]} and ${nodes[j]} for topic ${topic}`);
                    }
                }
                }
            }

            console.log("Edge data:", edgeData);
            // create the graph
            const data = {
                nodes: new DataSet(nodeData),
                edges: new DataSet(edgeData),
            };
            const container = document.getElementById("mynetwork");
            container.style.height = "600px";
            const options = {};
            const network = new Network(container, data, options);
            });
        });
        }

</script>


<div id="mynetwork"></div>


<!-- function updateGraph() {
const nodeIndex = {};
const nodes = [];
const edges = [];

// create nodes based on topics
nodeDetails.forEach((node) => {
    node.publishing.forEach((topic) => {
    if (!nodes.includes(topic)) {
        nodes.push(topic);
        nodeIndex[topic] = nodes.length;
    }
    });
    node.subscribing.forEach((topic) => {
    if (!nodes.includes(topic)) {
        nodes.push(topic);
        nodeIndex[topic] = nodes.length;
    }
    });
});
    console.log(nodeIndex);
// create edges based on matching topics
nodeDetails.forEach((node1) => {
    nodeDetails.forEach((node2) => {
    if (node1 != node2) {
        node1.publishing.forEach((topic1) => {
        node2.subscribing.forEach((topic2) => {
            if (topic1 == topic2) {
            edges.push({
                from: nodeIndex[topic1],
                to: nodeIndex[topic2],
                label: `${node1.name} → ${node2.name}`
            });
            }
        });
        });
    }
    });
}); -->