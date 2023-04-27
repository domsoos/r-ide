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

    function rosReconnect(){
        ROS.reconnect().then(() => {
            rosApi = ROS.getROSApi();
            getROSNodes();
        }).catch((err) => {
            isConnected = false;
            vscode.postMessage({
                type: 'r-ide.noConnection',
			});
            //console.error(err);
        });
    }

    
    function clearAllData(){

    }

    function getROSNodes(){
        isLoading = true;

        if(rosApi?.isConnected){
            isConnected = true;
            rosApi.getNodes((res) => {
                if (res) {
                    
                    getNodeDetails(res);
                    
                } else {
                    
                    isConnected = false;
                }
                isLoading = false;
            }, (err)=>{
                
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
                    
                    for (let subscribingNode of subscribingNodes) {
                        edgeData.push({
                            from: nodeDataItem.id,
                            to: subscribingNode.id,
                            label: topic,
                            length: 300
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

    function refreshNodeGraph(){
        if(isConnected){
            getROSNodes()
        }else{
            rosReconnect()
        }
    }

</script>

<div style="display: flex;width: 100%;justify-content: center;flex-direction: row;padding: 20px;">
    <h1 style="text-align: center;">Node Graph</h1>
    <button style="background: none!important;width:30px;height:30px;margin-left: 10px;" on:click={()=>{refreshNodeGraph()}}>
        <svg xmlns="http://www.w3.org/2000/svg" width="26" height="26" viewBox="0 0 16 16"><path d="M13.451 5.609l-.579-.939-1.068.812-.076.094c-.335.415-.927 1.341-1.124 2.876l-.021.165.033.163.071.345c0 1.654-1.346 3-3 3-.795 0-1.545-.311-2.107-.868-.563-.567-.873-1.317-.873-2.111 0-1.431 1.007-2.632 2.351-2.929v2.926s2.528-2.087 2.984-2.461h.012l3.061-2.582-4.919-4.1h-1.137v2.404c-3.429.318-6.121 3.211-6.121 6.721 0 1.809.707 3.508 1.986 4.782 1.277 1.282 2.976 1.988 4.784 1.988 3.722 0 6.75-3.028 6.75-6.75 0-1.245-.349-2.468-1.007-3.536z" fill="#2D2D30"/><path d="M12.6 6.134l-.094.071c-.269.333-.746 1.096-.91 2.375.057.277.092.495.092.545 0 2.206-1.794 4-4 4-1.098 0-2.093-.445-2.817-1.164-.718-.724-1.163-1.718-1.163-2.815 0-2.206 1.794-4 4-4l.351.025v1.85s1.626-1.342 1.631-1.339l1.869-1.577-3.5-2.917v2.218l-.371-.03c-3.176 0-5.75 2.574-5.75 5.75 0 1.593.648 3.034 1.695 4.076 1.042 1.046 2.482 1.694 4.076 1.694 3.176 0 5.75-2.574 5.75-5.75-.001-1.106-.318-2.135-.859-3.012z" fill="#C5C5C5"/></svg>
    </button>
</div>

<hr>

    
   

<div id="mynetwork"></div>
