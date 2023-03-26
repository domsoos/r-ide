<script>
    const vscode = acquireVsCodeApi();
    import TreeView from "./TreeView.svelte";
    import MessageTree from "./messageTree.svelte";
    import ROS from "../../ROSManagers/rosmanager"
    import { onMount } from 'svelte';
    import { Buffer } from "buffer";
    import { decodeBGRA8,
            decodeYUV,
            decodeYUYV,
            decodeRGB8,
            decodeRGBA8,
            decodeBGR8,
            decodeFloat1c,
            decodeMono8,
            decodeMono16,
            decodeBayerRGGB8,
            decodeBayerBGGR8,
            decodeBayerGBRG8,
            decodeBayerGRBG8 } from '../../utils/decoders'
    import ROSLIB from "roslib";

    let rosApi;

    let isConnected = true;
    let isLoading = false;
    let autoUpdateInterval;
    let topicsStatus = "Loading..."

    let topics = [];
    let mediaTopics = [];
    let activeMediaTopic = null;
    
    // holding list of subscirbed topics so i can compare against
    // new list and keep box checked
    let subscribedTopics = [];
    let expandedTopics = [];

    // Actual list of active subscriptions
    let activeSubcriptions = [];


    // Publisher Variables
    let selectedTopicToPublish = [];
    let messagePublisherFreq = 1;
    let publisherTimeInterval;

    function clearAllData(){
        topics = [];
        mediaTopics = [];
        activeMediaTopic = null;
        subscribedTopics = [];
        activeSubcriptions = [];
        expandedTopics = [];
    }

    onMount(async () => {

        try {
            await new ROS();
            rosApi = ROS.getROSApi();
            getROSTopics();
        } catch (err) {
            isConnected = false;
            vscode.postMessage({
                type: 'r-ide.noConnection',
			});
            //console.error(err);
        }
    });

    function rosReconnect(){
        clearAllData();
        isLoading = true;
        isConnected = true;
        ROS.reconnect().then(() => {
            rosApi = ROS.getROSApi();
            getROSTopics();
        }).catch((err) => {
            isConnected = false;
            vscode.postMessage({
                type: 'r-ide.noConnection',
			});
            //console.error(err);
        });
    }


    function getROSTopics(){
        isLoading = true;
        topicsStatus = "Loading...";

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

    //ROS topics autoupdate
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
            }
        }, 3000);
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
    }

    window.addEventListener('message', event => {
		const message = event.data; // The JSON data our extension sent
		switch (message.type) {
            case 'example':{
                console.log(message.data);
                break;
            }
            case 'setPublishMessageFormat': {
                onSetPublishMessageFormat(message.data);
                break;
            }
		}
	});

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
                        let isExpanded;
                        if(expandedTopics?.length > 0){
                            isExpanded = expandedTopics.find(item => item.topic === parts[i]);
                        }
                        node = {topic: parts[i], children: [], expanded: isExpanded? true : false};
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

        let newTopic = new ROSLIB.Topic({
                ros : rosApi,
                name : item.fulltopic,
                messageType : item.type
            });

        newTopic.subscribe((message) => {
            setRecentMessage(message, item.fulltopic);
        });

        activeSubcriptions.push(newTopic);
        subscribedTopics = [...subscribedTopics, item];
    }

    function setRecentMessage(message, topic){
        let myTopic = subscribedTopics.find(item => item.fulltopic === topic);
        myTopic.recentMessage = message
        // Figure out a better way to update the DOM
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
    }

    function subcribeToMediaTopic(item){
        if(activeMediaTopic !== null){
            activeMediaTopic.unsubscribe();
            activeMediaTopic = null;
        }

        // rosApi.getMessageDetails(item.type, (details) => {
        //     console.log(rosApi.decodeTypeDefs(details));
        // }, (error) => {
        //     console.log(error);
        // });
    
        let newMediaTopic = new ROSLIB.Topic({
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

        // console.log(message);

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
                
                /*
                if(width > height){
                    canvas.style.width = '500px';
                    canvas.style.height = 'fit-content';
                }
                else{
                    canvas.style.height = '250px';
                    canvas.style.width = 'fit-content';
                }
                */
                
                canvas.style.height = '250px';
                canvas.style.width = 'fit-content';
                

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
        value: value ? value.toString() : typeof value === "number" ? 0 : "null",
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

    function itemExpanded(item){
        if(item.expanded){
            expandedTopics.push(item);
        }
        else{
            let index = expandedTopics.findIndex((obj) => obj.topic === item.topic)
            expandedTopics.splice(index, 1);
        }   
    }

    function onPublishTopicSelected(item){
        let newItem = structuredClone(item);
        selectedTopicToPublish = [newItem];

        vscode.postMessage({
            type: "getMessageTypeFormat",
            value: newItem
        });
    }

    function onSetPublishMessageFormat(object){
        const messageElement =  document.getElementById('message-textarea');
        /*
        messageElement.value = "# Topic: " + selectedTopicToPublish[0].topic + "\n";
        messageElement.value += "# Type: " + selectedTopicToPublish[0].type + "\n";
        */

        messageElement.value = JSON.stringify(object, null, 2);
    }

    function publishMessage(){
        try{
            let myTopic = activeSubcriptions.find(item => item.name == selectedTopicToPublish[0].fulltopic);

            if(myTopic){

                const messageElement =  document.getElementById('message-textarea');
                const jsonString = messageElement.value;
                const jsonObject = JSON.parse(jsonString);

                var message = new ROSLIB.Message(jsonObject);
                let freq = hzToMs(messagePublisherFreq);

                if(publisherTimeInterval){
                    clearInterval(publisherTimeInterval);
                    publisherTimeInterval = null;
                }

                publisherTimeInterval = setInterval(() => {
                    myTopic.publish(message);
                }, freq);
                
            }else{
                /*
                let item = topics.find(item => item.fulltopic == selectedTopicToPublish[0].fulltopic);
                item.checked = true;
                updateCheckboxes(item);
                myTopic = activeSubcriptions.find(item => item.name == selectedTopicToPublish[0].fulltopic);
                */

                /*
                console.log(topics);
                let item = topics.find(item => item.fulltopic == selectedTopicToPublish[0].fulltopic);
                myTopic = new rosLib.Topic({
                    ros : rosApi,
                    name : item.fulltopic,
                    messageType : item.type
                });

                myTopic.subscribe((message) => {
                    setRecentMessage(message, item.fulltopic);
                });

                activeSubcriptions.push(myTopic);
                subscribedTopics = [...subscribedTopics, item];
                */
            }

        }catch(err){
            console.log(err);
        }


    }

    function stopPublish(){
        if(publisherTimeInterval){
            clearInterval(publisherTimeInterval);
            publisherTimeInterval = null;
        }
    }

    function hzToMs(freqHz) {
         return 1 / freqHz * 1000;
    }

</script>


<div class="container">
        <div class="topic-container">
            {#if isConnected}
                <div style="display:flex;align-items:center;justify-content: space-between;">
                    <h2 style="text-align: center;flex-grow: 1;margin-left: 25px;"><b>Status:</b> {topicsStatus}</h2>
                    <button class="refresh-btn" on:click={()=>{getROSTopics()}}>
                        <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16"><path d="M13.451 5.609l-.579-.939-1.068.812-.076.094c-.335.415-.927 1.341-1.124 2.876l-.021.165.033.163.071.345c0 1.654-1.346 3-3 3-.795 0-1.545-.311-2.107-.868-.563-.567-.873-1.317-.873-2.111 0-1.431 1.007-2.632 2.351-2.929v2.926s2.528-2.087 2.984-2.461h.012l3.061-2.582-4.919-4.1h-1.137v2.404c-3.429.318-6.121 3.211-6.121 6.721 0 1.809.707 3.508 1.986 4.782 1.277 1.282 2.976 1.988 4.784 1.988 3.722 0 6.75-3.028 6.75-6.75 0-1.245-.349-2.468-1.007-3.536z" fill="#2D2D30"/><path d="M12.6 6.134l-.094.071c-.269.333-.746 1.096-.91 2.375.057.277.092.495.092.545 0 2.206-1.794 4-4 4-1.098 0-2.093-.445-2.817-1.164-.718-.724-1.163-1.718-1.163-2.815 0-2.206 1.794-4 4-4l.351.025v1.85s1.626-1.342 1.631-1.339l1.869-1.577-3.5-2.917v2.218l-.371-.03c-3.176 0-5.75 2.574-5.75 5.75 0 1.593.648 3.034 1.695 4.076 1.042 1.046 2.482 1.694 4.076 1.694 3.176 0 5.75-2.574 5.75-5.75-.001-1.106-.318-2.135-.859-3.012z" fill="#C5C5C5"/></svg>
                    </button>
                </div>
            {:else}
                <div style="display:flex;align-items:center;justify-content: space-between;">
                    <h2 style="text-align: center;flex-grow: 1;margin-left: 25px;"><b>Error:</b> No connection</h2>
                    <button class="refresh-btn" on:click={()=>{rosReconnect()}}>
                        <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16"><path d="M13.451 5.609l-.579-.939-1.068.812-.076.094c-.335.415-.927 1.341-1.124 2.876l-.021.165.033.163.071.345c0 1.654-1.346 3-3 3-.795 0-1.545-.311-2.107-.868-.563-.567-.873-1.317-.873-2.111 0-1.431 1.007-2.632 2.351-2.929v2.926s2.528-2.087 2.984-2.461h.012l3.061-2.582-4.919-4.1h-1.137v2.404c-3.429.318-6.121 3.211-6.121 6.721 0 1.809.707 3.508 1.986 4.782 1.277 1.282 2.976 1.988 4.784 1.988 3.722 0 6.75-3.028 6.75-6.75 0-1.245-.349-2.468-1.007-3.536z" fill="#2D2D30"/><path d="M12.6 6.134l-.094.071c-.269.333-.746 1.096-.91 2.375.057.277.092.495.092.545 0 2.206-1.794 4-4 4-1.098 0-2.093-.445-2.817-1.164-.718-.724-1.163-1.718-1.163-2.815 0-2.206 1.794-4 4-4l.351.025v1.85s1.626-1.342 1.631-1.339l1.869-1.577-3.5-2.917v2.218l-.371-.03c-3.176 0-5.75 2.574-5.75 5.75 0 1.593.648 3.034 1.695 4.076 1.042 1.046 2.482 1.694 4.076 1.694 3.176 0 5.75-2.574 5.75-5.75-.001-1.106-.318-2.135-.859-3.012z" fill="#C5C5C5"/></svg>
                    </button>
                </div>
                <div class="connection-error">Start/Restart ROS bridge and click refresh</div>
            {/if}
            <hr>
            
            {#if topics.length > 0}
                <div class="topics">
                    <TreeView topics={topics} updateCheckboxes={updateCheckboxes} vscode={vscode} onExpand={itemExpanded} publishTopicSelected={onPublishTopicSelected}/>   
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
                            {:else}
                                <h3>No data</h3>    
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
                        <button class={activeMediaTopic?.name == item.topic ? 'isSelected' : ''} on:click={() => {subcribeToMediaTopic(item)}}>{item.topic}</button>
                    {/each}
                </div>
              </div>
            <div class="canvas-container">
                <canvas class="img-canvas" id="my-canvas"></canvas>
            </div>

            <div class="message-publisher-container">
                <h2 style="text-align: center;">Message Publisher</h2>
                <hr>
                {#each selectedTopicToPublish as item}
                    <!-- svelte-ignore a11y-click-events-have-key-events -->
                    <!--
                    <div class="message-publisher-block">
                        <span><b style="color:white">Topic : </b>{item.topic}</span>
                        <br>
                        <span><b style="color:white">Type : </b>{item.type}</span>
                    </div>
                    -->
                    <div class="message-publisher-data">
                        <div style="display:flex;justify-content:space-between;">
                            <div>
                                <b style="color:white">Topic : </b>{item.topic}
                                <br>
                                <b style="color:white">Type : </b>{item.type}
                            </div>
                            <div style="display:flex;align-items:center;">
                                <b style="margin-right:10px;">Freq(hz) : </b>
                                <input type="number" bind:value={messagePublisherFreq} style="width: 50px">
                            </div>
                        </div>


                        <textarea disabled={publisherTimeInterval} id="message-textarea" style="height:165px;resize: none;margin-top:5px"></textarea>
                        {#if !publisherTimeInterval}
                            <button class="publish-button" disabled={messagePublisherFreq <= 0 || messagePublisherFreq > 100} on:click={() => {publishMessage()}}>Publish</button>
                        {:else}
                            <button class="publish-button" on:click={() => {stopPublish()}}>Stop</button>
                        {/if}
                    </div>
                {/each}
            </div>
            <!--<button on:click={()=>{vscode.postMessage({type: 'getMessageTypes'});}}>get message types</button>-->
        </div>
</div>

