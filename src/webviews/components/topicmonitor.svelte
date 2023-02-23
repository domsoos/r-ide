<script>
    const vscode = acquireVsCodeApi();
    import TreeView from "./TreeView.svelte";
    //import * as ROSLIB from "roslib";
    import ROS from "../../ROSManagers/rosmanager.js"
    import { onMount } from 'svelte';

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
        }, 5000);
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
        if(message.encoding === "bgra8"){
            const binaryData = atob(message.data);
            const bytes = new Uint8Array(binaryData.length);
            for (let i = 0; i < binaryData.length; i++) {
                bytes[i] = binaryData.charCodeAt(i);
            }
            const imageData = new ImageData(new Uint8ClampedArray(bytes.buffer), message.width, message.height);
            const canvas = document.getElementById('my-canvas');
            canvas.width = message.width;
            canvas.height = message.height;

            if(message.width > message.height){
                canvas.style.width = '500px';
                canvas.style.height = 'fit-content';
            }
            else{
                canvas.style.height = '350px';
                canvas.style.width = 'fit-content';
            }

            const context = canvas.getContext('2d');
            context.putImageData(imageData, 0, 0);

        }else if(message.encoding === "bayer_rggb8"){
                const binaryData = atob(message.data);
                const bytes = new Uint8Array(binaryData.length);
                for (let i = 0; i < binaryData.length; i++) {
                    bytes[i] = binaryData.charCodeAt(i);
                }
                const width = message.width;
                const height = message.height;
                
                const pixels = parseBayerData(bytes, width, height);
                const imageData = createImageData(pixels, width, height);

                const canvas = document.getElementById('my-canvas');
                canvas.width = width;
                canvas.height = height;

                if(message.width > message.height){
                    canvas.style.width = '500px';
                    canvas.style.height = 'fit-content';
                }
                else{
                    canvas.style.height = '350px';
                    canvas.style.width = 'fit-content';
                }

                const context = canvas.getContext('2d');
                context.putImageData(imageData, 0, 0);
        }
    }


    function parseBayerData(data, width, height) {
        const pixelCount = width * height;
        const pixels = new Array(pixelCount);

        for (let i = 0; i < pixelCount; i++) {
            const r = data[i];
            const g = data[i + 1];
            const b = data[i + 2];

            // Convert R, G, B values to grayscale value
            const grayValue = (r + g + b) / 3;

            // Calculate row and column indices for current pixel
            const row = Math.floor(i / width);
            const col = i % width;

            // Calculate the position of the current pixel in the output array
            const outputIndex = (row * width) + col;

            // Set the pixel value in the output array
            pixels[outputIndex] = grayValue;
        }

        return pixels;
    }

    function createImageData(pixels, width, height) {
        const imageData = new ImageData(new Uint8ClampedArray(width * height * 4), width, height);
        const pixelData = imageData.data;

        for (let i = 0; i < pixels.length; i++) {
            const outputIndex = i * 4;
            pixelData[outputIndex] = pixels[i];
            pixelData[outputIndex + 1] = pixels[i];
            pixelData[outputIndex + 2] = pixels[i];
            pixelData[outputIndex + 3] = 255; // Set alpha value to 255
        }

        return imageData;
    }

    function displayObjectProperties(obj, prefix = '') {
        let messageData = [];
        for (const key in obj) {
            const value = obj[key];
            const propertyKey = prefix ? `${prefix}.${key}` : key;

            let data = {
                property: propertyKey,
                value: value ? value.toString(): "null"
            }
            
            //console.log(`${propertyKey}: ${typeof value}`);
            
            if (typeof value === 'object') {
            displayObjectProperties(value, propertyKey);
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
                            <div class="message-data">
                                {#if item.recentMessage}
                                    {#each displayObjectProperties(item.recentMessage) as messageData}
                                        <div>{messageData.property}: {messageData.value}</div>
                                    {/each}
                                {/if}
                            </div>
                        {:else}
                            <span style="float: left;margin-right: 15px">&#x25b6</span>{item.fulltopic}
                        {/if}
                            
                    </div>
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

