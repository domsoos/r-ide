<script>
    const vscode = acquireVsCodeApi();/*
    import TreeView from "./TreeView.svelte";

    let isConnected = null;
    let isLoading = false;
    let topics = [];
    let buttonLabel = 'View Topics'; 

    window.addEventListener('message', event => {
		const message = event.data; // The JSON data our extension sent
		switch (message.type) {
			case 'setROSTopics': {
                topics = [];
                if(message.success && message.data.topics){
                    let temp = []
                    for (let i = 0; i < message.data.topics.length; i++){
                        temp.push({
                            topic: message.data.topics[i],
                            type: message.data.types[i],
                        });
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
            case 'imgTest':{
                const binaryData = atob(message.data.data);
                const bytes = new Uint8Array(binaryData.length);
                for (let i = 0; i < binaryData.length; i++) {
                    bytes[i] = binaryData.charCodeAt(i);
                }
                const imageData = new ImageData(new Uint8ClampedArray(bytes.buffer), message.data.width, message.data.height);
                const canvas = document.getElementById('my-canvas');
                canvas.width = message.data.width;
                canvas.height = message.data.height;
                const context = canvas.getContext('2d');
                context.putImageData(imageData, 0, 0);
                break;
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

    function updateCheckboxes() {
        topics = [...topics];
    }
    */

    function openTopicMonitor(){
        vscode.postMessage({
            type: 'openTopicMonitor',
        });
    };
</script>


<button on:click={() => {openTopicMonitor()}}>ROS Topic Monitor</button>
<button>ROS Topic Publisher</button>

<!--
<button disabled='{isLoading}'  on:click={() => {getROSTopics();}}>{buttonLabel}</button>

{#if topics.length > 0}
    <div style="display: flex; justify-content: space-between;">
        <p style="padding-left: 1.2rem;margin-top: 5px">Topics</p>
        svelte-ignore a11y-click-events-have-key-events
        <p style="font-size: 20px; margin-right: 2px; cursor:pointer;user-select: none;" on:click={()=>{clearAll()}}>&#9746;</p>
    </div>
    <hr>

    <TreeView topics={topics} updateCheckboxes={updateCheckboxes} vscode={vscode}/>   

{/if}

<canvas id="my-canvas" style="height: 300px;"></canvas>
-->
