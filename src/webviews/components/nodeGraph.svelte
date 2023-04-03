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

    let nodes = [];


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
                    getNodeDetails(res);
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

    function getNodeDetails(nodes){
        for(let node of nodes){
            rosApi.getNodeDetails(node, (res) => {
                console.log(res);
            })
        }

        const mynodes = new DataSet([
            { id: 1, label: "Node 1" },
            { id: 2, label: "Node 2" },
            { id: 3, label: "Node 3" },
            { id: 4, label: "Node 4" },
            { id: 5, label: "Node 5" }
        ]);

        const edges = new DataSet([
            { from: 1, to: 3 },
            { from: 1, to: 2 },
            { from: 2, to: 4 },
            { from: 2, to: 5 },
            { from: 3, to: 3 }
        ]);

        const container = document.getElementById("mynetwork");
        container.style.height = "600px";

        const data = {
            nodes: mynodes,
            edges: edges
        };

        const options = {};
        const network = new Network(container, data, options);
    }

</script>


<div id="mynetwork"></div>

