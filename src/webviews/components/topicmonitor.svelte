<script>
    const vscode = acquireVsCodeApi();
    import TreeView from "./TreeView.svelte";

    let isConnected = null;
    let isLoading = false;
    let topics = [];
    let buttonLabel = 'View Topics'; 
    let currentMediaTopic = "None"
    let mediaTopics = [];
    

    window.addEventListener('message', event => {
		const message = event.data; // The JSON data our extension sent
		switch (message.type) {
			case 'setROSTopics': {
                topics = [];
                mediaTopics = [];
                if(message.success && message.data.topics){
                    let temp = []
                    for (let i = 0; i < message.data.topics.length; i++){
                        if(message.data.types[i] == "sensor_msgs/Image"){
                            mediaTopics = [...mediaTopics, {
                                topic: message.data.topics[i],
                                type: message.data.types[i],
                            }]
                        }
                        else{
                            temp.push({
                                topic: message.data.topics[i],
                                type: message.data.types[i],
                            });
                        }
                    }
                    topics = buildTree(temp);
                    buttonLabel = 'Refresh Topics';
                    isConnected = true;
                    isLoading = false;
                }else if(!message.success && message.data){
                    vscode.postMessage({
                        type: 'onError',
                        value: message.data + "- Please try restarting ROSBridge."
                    });
                    buttonLabel = 'No connection. Try again.'
                    isConnected = false;
                    isLoading = false;
                }
                else{
                    vscode.postMessage({
                        type: 'onError',
                        value: "Cannot communicate with ROS. Please ensure ROSBridge is running."
                    });
                    buttonLabel = 'No connection. Try again.'
                    isConnected = false;
                    isLoading = false;
                }
                break;
            }
            case 'setActiveMediaTopic':{
                if(message.data.encoding === "bgra8"){
                    const binaryData = atob(message.data.data);
                    const bytes = new Uint8Array(binaryData.length);
                    for (let i = 0; i < binaryData.length; i++) {
                        bytes[i] = binaryData.charCodeAt(i);
                    }
                    const imageData = new ImageData(new Uint8ClampedArray(bytes.buffer), message.data.width, message.data.height);
                    const canvas = document.getElementById('my-canvas');
                    canvas.width = message.data.width;
                    canvas.height = message.data.height;

                    if(message.data.width > message.data.height){
                        canvas.style.width = '500px';
                        canvas.style.height = 'fit-content';
                    }
                    else{
                        canvas.style.height = '350px';
                        canvas.style.width = 'fit-content';
                    }

                    const context = canvas.getContext('2d');
                    context.putImageData(imageData, 0, 0);

            
                }else if(message.data.encoding === "bayer_rggb8"){

                    const binaryData = atob(message.data.data);
                    const bytes = new Uint8Array(binaryData.length);
                    for (let i = 0; i < binaryData.length; i++) {
                        bytes[i] = binaryData.charCodeAt(i);
                    }
                    const width = message.data.width;
                    const height = message.data.height;
                    
                    const pixels = parseBayerData(bytes, width, height);
                    const imageData = createImageData(pixels, width, height);

                    const canvas = document.getElementById('my-canvas');
                    canvas.width = width;
                    canvas.height = height;

                    if(message.data.width > message.data.height){
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

                break;
            }
            case 'messageFromTopic':{
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
                        node = {topic: parts[i], children: [], fulltopic: topic.topic, type: topic.type, checked: false};
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

    function updateCheckboxes() {
        topics = [...topics];
    }

    function subcribeToMediaTopic(item){
        currentMediaTopic = item.topic;
        vscode.postMessage({
            type: 'subActiveMediaTopic',
            value: {topic: item.topic, type: item.type}
        });
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

</script>


<div class="container">
        <div class="topic-container">
            <button disabled='{isLoading}'  on:click={() => {getROSTopics();}}>{buttonLabel}</button>
        
            {#if topics.length > 0}
                <div class="topics">
                    <TreeView topics={topics} updateCheckboxes={updateCheckboxes} vscode={vscode}/>   
                </div>
            {/if}
        </div>
        <div class="messages-container">
            <h2 style="text-align: center;">Messages</h2>
            <hr>
        </div>

        <div class="media-column">
            <div class="dropdown">
                <button class="dropbtn"><b>Selected Media:</b> {currentMediaTopic}<span style="float: right;">&#x25BC</span></button>
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

