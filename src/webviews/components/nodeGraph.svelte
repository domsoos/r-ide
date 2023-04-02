<script>
    const vscode = acquireVsCodeApi();
    import { onMount } from 'svelte';
    import ROS from "../../ROSManagers/rosmanager"
    import ROSLIB from "roslib";

    let rosApi;

    let isLoading = false;


    onMount(async () => {

    try {
        await new ROS();
        rosApi = ROS.getROSApi();
        //getROSNodes();
    } catch (err) {
        isConnected = false;
        vscode.postMessage({
            type: 'r-ide.noConnection',
        });
        //console.error(err);
    }
    });

    /*
    function getROSNodes(){
        isLoading = true;

        if(rosApi?.isConnected){
            rosApi.getTopics((res) => {
                if (res) {
                    updateTopicTree(res);
                    topicsStatus = "Connected";
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
    */


</script>

