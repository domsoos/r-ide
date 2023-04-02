<script>
    const vscode = acquireVsCodeApi();
    import { onMount } from 'svelte';
    import ROS from "../../ROSManagers/rosmanager"
    import ROSLIB from "roslib";

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
    }

</script>

