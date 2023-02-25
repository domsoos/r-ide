<script>
    const vscode = acquireVsCodeApi();
    import TreeView from "./TreeView.svelte";
    import MessageTree from "./messageTree.svelte";
    //import * as ROSLIB from "roslib";
    import ROS from "../../ROSManagers/rosmanager.js"
    import { onMount } from 'svelte';
    import { Buffer } from "buffer";
    import { decodeBGRA8,
             decodeMono8,
             decodeFloat1c,
             decodeBayerRGGB8,
             decodeBayerBGGR8,
             decodeBayerGBRG8,
             decodeBayerGRBG8 } from '../../utils/decoders'

    let ros;
    let rosApi;
    let rosLib;

    let isConnected = null;
    let isLoading = false;
    //let buttonLabel = 'View Topics'; 
    //let currentMediaTopic = "None"
    let autoUpdateInterval;
    let topicsStatus = "No connection -  Please ensure ROSBridge is running"

    let topics = [];
    let mediaTopics = [];
    let activeMediaTopic = null;
    
    // holding list of subscirbed topics so i can compare against
    // new list and keep box checked
    let subscribedTopics = [];

    // Actual list of active subscriptions
    let activeSubcriptions = [];




    onMount(async () => {
        ros = new ROS();
        rosApi = ROS.getROSApi();
        rosLib = ROS.getRosLib();

        autoUpdateTopics();
    });


    function autoUpdateTopics(){
        autoUpdateInterval = setInterval(() => {
            if(rosApi.isConnected){
                rosApi.getTopics((res) => {
                    if (res) {
                        updateTopicTree(res);
                    } else {
                        console.log("failed")
                    }
                }, (err)=>{
                    console.log(err);
                });
                clearInterval(autoUpdateInterval);
            }
        }, 1000);
    }

    function updateTopicTree(rawTopics){
        topics = [];
        mediaTopics = [];
        let temp = [];
        for (let i = 0; i < rawTopics.topics.length; i++){
            if(rawTopics.types[i] == "sensor_msgs/Image"){
                mediaTopics = [...mediaTopics, {
                    topic: rawTopics.topics[i],
                    type: rawTopics.types[i],
                }]
            }
            else{
                let subbed = subscribedTopics.find(item => item.fulltopic === rawTopics.topics[i]);
                temp.push({
                    topic: rawTopics.topics[i],
                    type: rawTopics.types[i],
                    checked: subbed ? true : false
                });
            }
        }
        topics = buildTree(temp);
        isConnected = true;
        topicsStatus = "Connection Successful"
    }

    window.addEventListener('message', event => {
		const message = event.data; // The JSON data our extension sent
		switch (message.type) {
            case 'example':{
                console.log(message.data);
            }
		}
	});
    

    function getROSTopics(){
        isLoading = true;
        vscode.postMessage({
            type: 'getROSTopics',
        });

        setTimeout(() => {
            if(isLoading)
            isLoading = false;
        }, 3000);
    }

    function buildTree(topics) {
        const root = {topic: "/", children: []};
        const map = {[root.topic]: root};


        topics.forEach(topic => {
            const parts = topic.topic.split("/").filter(Boolean);
            let parent = root;
            let path = "";

            for (let i = 0; i < parts.length;i++){
                path += "/" + parts[i];
                let node = map[path];
                if (!node) {
                    if(i == parts.length - 1){
                        node = {topic: parts[i], children: [], fulltopic: topic.topic, type: topic.type, checked: topic.checked};
                    }
                    else{
                        node = {topic: parts[i], children: []};
                    }
                    map[path] = node;
                if (parent) {
                parent.children.push(node);
                }
                parent = node;
                } 
                else {
                    parent = node;
                }
            }
        });

        return root.children;
    }

    /*
    function clearAllChecks(element) {
        if (element.fulltopic && element.checked) {
            element.checked = false;
        }
        if (element.children.length > 0) {
            element.children.forEach(child => {
            clearAllChecks(child);
        });
        }
    }

    function clearAll() {
        topics.forEach(item => {
            clearAllChecks(item);
        });

        updateCheckboxes();
    }
    */

    function updateCheckboxes(item) {
        if(item.checked){
            subscribeToTopic(item);
        }
        else{
            unsubscribeFromTopic(item);
        }
        
        //topics = [...topics];
    }

    function subscribeToTopic(item){

        let newTopic = new rosLib.Topic({
                ros : rosApi,
                name : item.fulltopic,
                messageType : item.type
            });

        newTopic.subscribe((message) => {
            setRecentMessage(message, item.fulltopic);
        });

        activeSubcriptions.push(newTopic);
        //subscribedTopics.push(item);
        subscribedTopics = [...subscribedTopics, item];
        //console.log(subscribedTopics);
        //console.log(activeSubcriptions);
    }

    function setRecentMessage(message, topic){
        let myTopic = subscribedTopics.find(item => item.fulltopic === topic);
        myTopic.recentMessage = message
        // NEED BETTER WAY TO UPDATE DOM
        subscribedTopics = [...subscribedTopics];
    }

    function unsubscribeFromTopic(item){
        let index = subscribedTopics.findIndex((obj) => obj.fulltopic === item.fulltopic);
        subscribedTopics.splice(index, 1);
        // Figure out a better way to update the DOM
        subscribedTopics = [...subscribedTopics];

        index = activeSubcriptions.findIndex((obj) => obj.name === item.fulltopic)
        activeSubcriptions[index].unsubscribe();
        activeSubcriptions.splice(index, 1);

        //console.log(subscribedTopics);
        //console.log(activeSubcriptions);
    }

    function subcribeToMediaTopic(item){
        if(activeMediaTopic !== null){
            activeMediaTopic.unsubscribe();
            activeMediaTopic = null;
        }
    
        let newMediaTopic = new rosLib.Topic({
            ros : rosApi,
            name : item.topic,
            messageType : item.type
        });

        newMediaTopic.subscribe( (message)=> {
            decodeImageMessage(message);
        });

        activeMediaTopic = newMediaTopic;
    }

    function decodeImageMessage(message){

        if(message.data){
            let width = message.width;
            let height = message.height;
            let binaryString = Buffer.from(message.data, 'base64');
            let rawData = new Uint8Array(binaryString);
            const image = new ImageData(width, height);

            if(message.encoding){
                switch(message.encoding){
                    case "yuv422":
                        decodeYUV(rawData, width, height, image.data);
                        break;
                    // same thing as yuv422, but a distinct decoding from yuv422 and yuyv
                    case "uyuv":
                        decodeYUV(rawData, width, height, image.data);
                        break;
                    // change name in the future
                    case "yuyv":
                        decodeYUYV(rawData, width, height, image.data);
                        break;
                    case "rgb8":
                        decodeRGB8(rawData, width, height, image.data);
                        break;
                    case "rgba8":
                        decodeRGBA8(rawData, width, height, image.data);
                        break;
                    case "bgra8":
                        decodeBGRA8(rawData, width, height, image.data);
                        break;
                    case "bgr8":
                        case "8UC3":
                        decodeBGR8(rawData, width, height, image.data);
                        break;
                    case "32FC1":
                        decodeFloat1c(rawData, width, height, message.is_bigendian, image.data);
                        break;
                    case "bayer_rggb8":
                        decodeBayerRGGB8(rawData, width, height, image.data);
                        break;
                    case "bayer_bggr8":
                        decodeBayerBGGR8(rawData, width, height, image.data);
                        break;
                    case "bayer_gbrg8":
                        decodeBayerGBRG8(rawData, width, height, image.data);
                        break;
                    case "bayer_grbg8":
                        decodeBayerGRBG8(rawData, width, height, image.data);
                        break;
                    case "mono8":
                    case "8UC1":
                        decodeMono8(rawData, width, height, image.data);
                        break;
                    case "mono16":
                    case "16UC1":
                        decodeMono16(rawData, width, height, message.is_bigendian, image.data, options);
                        break;
                    default:
                        throw new Error(`Unsupported encoding ${encoding}`);
                }

                const canvas = document.getElementById('my-canvas');
                canvas.width = width;
                canvas.height = height;

                if(width > height){
                    canvas.style.width = '500px';
                    canvas.style.height = 'fit-content';
                }
                else{
                    canvas.style.height = '350px';
                    canvas.style.width = 'fit-content';
                }

                const context = canvas.getContext('2d');
                context.putImageData(image, 0, 0);
                }
        }
    }

    function displayObjectProperties(obj, prefix = '') {
    let messageData = [];
    for (const key in obj) {
        const value = obj[key];
        let propertyKey;

        if (Array.isArray(obj)) {
        propertyKey = `${prefix}[${key}]`;
        } else {
        propertyKey = key;
        }

        let data = {
        property: propertyKey,
        value: value ? value.toString() : "null",
        type: Array.isArray(value) ? "array" : typeof value
        }

        if (typeof value === 'object' && value !== null) {
        data.children = displayObjectProperties(value, key);
        data.type = "object";
        }

        messageData.push(data);
    }
    return messageData;
    }

</script>


<div class="container">
        <div class="topic-container">
            <h2 style="text-align: center;"><b>Status:</b> { topicsStatus }</h2>
            <hr>
            <!--<button disabled='{isLoading}'  on:click={() => {getROSTopics();}}>{buttonLabel}</button>-->
        
            {#if topics.length > 0}
                <div class="topics">
                    <TreeView topics={topics} updateCheckboxes={updateCheckboxes} vscode={vscode}/>   
                </div>
            {/if}
        </div>
        <div class="messages-container">
            <h2 style="text-align: center;">Messages</h2>
            <hr>
            <div class="message-block-container">
                {#each subscribedTopics as item, i}
                    <!-- svelte-ignore a11y-click-events-have-key-events -->
                    <div class="message-block" on:click={()=>{ item.expanded = !item.expanded}}>
                        {#if item.expanded}
                            <span style="float: left;margin-right: 15px">&#x25BC</span>{item.fulltopic}
                        {:else}
                            <span style="float: left;margin-right: 15px">&#x25b6</span>{item.fulltopic}
                        {/if}
                    </div>
                    {#if item.expanded}
                        <div class="message-data">
                            {#if item.recentMessage}
                                <MessageTree nodes={displayObjectProperties(item.recentMessage)}/>
                            {/if}
                        </div>
                    {/if}
                {/each}
            </div>
        </div>

        <div class="media-column">
            <div class="dropdown">
                <button class="dropbtn"><b>Selected Media:</b> {activeMediaTopic ? activeMediaTopic.name : "None"}<span style="float: right;">&#x25BC</span></button>
                <div class="dropdown-content">
                    {#each mediaTopics as item, i}
                        <button on:click={() => {subcribeToMediaTopic(item)}}>{item.topic}</button>
                    {/each}
                </div>
              </div>
            <div class="canvas-container">
                <canvas class="img-canvas" id="my-canvas"></canvas>
            </div>
        </div>
</div>

