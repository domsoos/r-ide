<script>
    const vscode = acquireVsCodeApi();
    import Slider from '@bulatdashiev/svelte-slider';
    import { slide } from "svelte/transition";

    /* Accordion Code */
    let isAccordionOpen = false
	const toggle = () => isAccordionOpen = !isAccordionOpen
    /* Accordion Code END*/

    let topics = [];
    let selectedTopic = [];
    let messages = [];
    let isPlaying = false;

    let recordMenuOpen = false;
    let isBagManagerOpen = false;
    let isRecording = false;
    let selectedBag = null;
    let selectedBagPath = null;
    let cloneBagPath = null;
    let isCloneMenuOpen = false;
    let messagesLoaded = false;
    let connectionsLoaded = false;
    let cloneName = null;
    let verbose = false;
    let bagDuration;
    let currentTime = 0;

    let recordBag = {
        topics: [],
        location: "",
        recordAll: false,
        regex: null,
        regexExclude: null,
        quiet: true,
        duration: null,
        prefix: null,
        name: null,
        split: null,
        maxSplits: null,
        bufferSize: null,
        node: null
    }
    
    let range = [0,1]; 

    window.addEventListener('message', event => {
        
		const message = event.data; // The JSON data our extension sent
		switch (message.type) {
			case 'setSelectedBag':{
                connectionsLoaded = false;
                messagesLoaded = false;

                

                bagDuration = message.value.duration;
                selectedBagPath = message.value.path;                
                selectedBag = selectedBagPath.substring(selectedBagPath.lastIndexOf('/'));
                cloneBagPath = selectedBagPath.substring(0, selectedBagPath.lastIndexOf('/'));
                
                range[1] = bagDuration;
                
				break;
            }
            case 'setCloneBagPath':{
                cloneBagPath = message.value;
				break;
            }
            case 'getConnections': {
                let connections = message.value;
                setupPublishers(connections);
                break;
            }
            case 'getMessages': {
                if (message.legnth === 0) {
                    messages = message.value;
                    
                } else {
                    messages.push(...message.value);
                    
                }
                break;
            }
            case 'createdConnections': {
                connectionsLoaded = true;
                topics = [...message.value];
                
                break;
            }
            case 'createdMessages': {
                messagesLoaded = true;
                
                break;
            }
            case "finishedPlaying": {
                isPlaying = false;
                break;
            }
            case "ROSConnectionSuccessful": {
                if (!isBagManagerOpen){
                   openBagManager(); 
                } else {
                    vscode.postMessage({type: 'playBag'}); 
                    isPlaying = true;
                }
                break;
            }
            case "publishedTopics": {
                topics = message.value;
                break;
            }
            case "stoppedRecording": {
                isRecording = false;
                break;
            }
            case "getRecordPath": {
                
                recordBag.location = message.value;
                break;
            }
            case "timeProgress": {
                currentTime = message.value;
                break;
            }
		}
	});

    function openBagManager(){
        isBagManagerOpen = true;
    }

    function isROSConnected(){
        vscode.postMessage({
            type: 'isROSConnected', 
        })
    }

</script>
<style>
    .play-menu {
        display: flex;
        justify-content: space-between;
        width: 100%;
        padding: 0 10px;
    }
    .icon-button {
      display: inline-flex;
      align-items: center;
      justify-content: center;
      width: 60px;
      height: 60px;
      padding: 0;
      border: none;
      background: none;
      cursor: pointer;
      border-radius: 50%;
      flex: 1;
      margin: 0 10px;
      text-align: center;
    }
  
    .icon {
      width: 100%;
      height: 100%;
    }
    .replay {
        width:80%;
        height:80%;
    }
  </style>


<h3 style="text-align:center;margin: 10px 0px;">ROS Bag Manager</h3>
<hr style="margin-bottom:10px">
<!-- Recording a new bag UNIMPLEMENTED -->
{#if !isBagManagerOpen}
    <button disabled='{isRecording}' on:click={() => {isROSConnected()}}>Manage Bags</button>
    <button on:click={() => {isROSConnected(); vscode.postMessage({type: "getPublishedTopics"});  recordMenuOpen = true;}}>Record a bag</button>
   

<!-- Managing existing bags -->
{:else if isBagManagerOpen && !recordMenuOpen}
    <!-- Get new bag -->
    <button on:click={() => {vscode.postMessage({type: 'getSelectedBag'})}}>{selectedBag == null? 'Select Bag..': 'Selected Bag: ...' + selectedBag}</button>

    <!-- Go back one screen -->
    {#if selectedBag === null}
        <button on:click={() => {isBagManagerOpen = false}}>Cancel</button>
    {/if}

    <!-- Manage selected bag -->
    {#if selectedBag !== null}
        <!--<div style="margin:5px 0px;">Selected Bag: .../{selectedBag}</div>-->
        <!-- Play bag -->
        <!-- {#if !isCloneMenuOpen} -->
        <!-- Play menu-->
        <div class="play-menu">
            {#if !isPlaying}
                <button class="icon-button" on:click={() => {isROSConnected(); vscode.postMessage({type: 'playBag'})}}>
                    <svg class="icon" fill="#000000" viewBox="0 0 24 24" id="play" data-name="Flat Color" xmlns="http://www.w3.org/2000/svg"><circle id="primary" cx="12" cy="12" r="10" style="fill: rgb(0, 0, 0);"></circle><path id="secondary" d="M14.75,12.83,11.55,15A1,1,0,0,1,10,14.13V9.87A1,1,0,0,1,11.55,9l3.2,2.13A1,1,0,0,1,14.75,12.83Z" style="fill: rgb(44, 169, 188);"></path></svg>
                </button>
            {:else}
                <button class="icon-button" on:click={() => {vscode.postMessage({type: 'pauseBag'}); isPlaying = false;}}>
                    <svg class="icon" fill="#000000" viewBox="0 0 24 24" id="pause-circle" data-name="Flat Color" xmlns="http://www.w3.org/2000/svg"><circle id="primary" cx="12" cy="12" r="10" style="fill: rgb(0, 0, 0);"></circle><path id="secondary" d="M14,17a1,1,0,0,1-1-1V8a1,1,0,0,1,2,0v8A1,1,0,0,1,14,17Zm-4,0a1,1,0,0,1-1-1V8a1,1,0,0,1,2,0v8A1,1,0,0,1,10,17Z" style="fill: rgb(44, 169, 188);"></path></svg>               
                </button>
            {/if}
            <button class="icon-button" on:click={() => {vscode.postMessage({type: 'stopBag'}); isPlaying=false; currentTime = 0;}}>
                <svg class="icon" fill="#000000" viewBox="0 0 24 24" id="stop-circle" data-name="Flat Color" xmlns="http://www.w3.org/2000/svg"><circle id="primary" cx="12" cy="12" r="10" style="fill: rgb(0, 0, 0);"></circle><rect id="secondary" x="9" y="9" width="6" height="6" rx="1" transform="translate(24) rotate(90)" style="fill: rgb(44, 169, 188);"></rect></svg>
            </button>
            <button class="icon-button" on:click={() => {vscode.postMessage({type:'replayBag'}); currentTime = 0;}}>
                <svg class="replay" fill="#000000" version="1.1" id="Layer_1" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" 
                    viewBox="0 0 300.003 300.003" xml:space="preserve">
                    <g>
                        <g>
                        <path d="M150.005,0C67.164,0,0.001,67.159,0.001,150c0,82.838,67.162,150.003,150.003,150.003S300.002,232.838,300.002,150
                            C300.002,67.159,232.844,0,150.005,0z M230.091,172.444c-9.921,37.083-43.801,64.477-83.969,64.477
                            c-47.93,0-86.923-38.99-86.923-86.923s38.99-86.92,86.923-86.92c21.906,0,41.931,8.157,57.228,21.579l-13.637,23.623
                            c-11-11.487-26.468-18.664-43.594-18.664c-33.294,0-60.38,27.088-60.38,60.38c0,33.294,27.085,60.38,60.38,60.38
                            c25.363,0,47.113-15.728,56.038-37.937h-20.765l36.168-62.636l36.166,62.641H230.091z"/>
                        </g>
                    </g>
                </svg>
            </button>
        </div>

        <div>
            <label for="progress">{currentTime}s / {bagDuration}s</label>
            <progress id="progress" max={bagDuration} value={currentTime}></progress>
        </div>

        {#if !isCloneMenuOpen}
        <!-- Cancel and Clone buttons -->
        <div class="buttons-flex">
            <button class="bag-buttons" on:click={() => {isBagManagerOpen = false; selectedBag = null; isPlaying = false; vscode.postMessage({type: "closeBag"}); currentTime = 0;}}>Cancel</button>
            <button class="bag-buttons" on:click={() => {isCloneMenuOpen = !isCloneMenuOpen; range[1] = currentTime + 1}}>Clone</button>
            <!-- <button class="bag-buttons" tooltip="Coming soon">Clone</button> -->
        </div>

        <!-- Clone bag menu -->
        {:else}
            <br>
            <hr>
            <br>
            <label for="bag_name"><b>New bag name:</b></label>

            <!-- Bag namne -->
            <input id="bag_name" type="text" class="margin-top-5" style="border:solid 1px black" bind:value={cloneName} placeholder="Enter new bag name...">
            <br>

            <!-- Bag location -->
            <label for="bag_location"><b>New bag location:</b></label>
            <input id="bag_location" type="text" class="margin-top-5 location-input" value="{cloneBagPath? cloneBagPath : "TODO::NEED TO FIX THIS"}" style="border:solid 1px black; width:88%">
            <button class="location-btn" on:click={() => {vscode.postMessage({type: 'getCloneBagPath', value:cloneBagPath ? cloneBagPath : selectedBagPath})}}>...</button>
            <br>
            <br>

            <!-- Clone trim -->
            <b>Trim clone:</b>
            <div class="buttons-flex">
                <input type="number" bind:value={range[0]} style="width: 50px">
                <input type="number" bind:value={range[1]} style="width: 50px">
            </div> 
            <Slider max={bagDuration} step={1} bind:value={range} range order style="margin-right:20px"/>

            <!-- Topics -->
            <button class="accordion-button" on:click={toggle} aria-expanded={isAccordionOpen}><svg style="tran"  width="20" height="20" fill="none" stroke-linecap="round" stroke-linejoin="round" stroke-width="2" viewBox="0 0 30 10" stroke="currentColor"><path d="M9 5l7 7-7 7"></path></svg><b>Filter Topics</b></button>
            
            {#if isAccordionOpen}
                <ul><button class="bag-buttons" on:click={() => {topics.length === selectedTopic.length ? selectedTopic = [] : selectedTopic = topics}}>Select all</button></ul>
                <ul style="list-style: none" transition:slide={{ duration: 300 }}>
                    {#each topics as item}
                        <li><label><input type=checkbox bind:group={selectedTopic} value={item}>{item}</label></li>
                    {/each}
                </ul>
            {/if}

            <br><label><input type=checkbox bind:checked={verbose}>Verbose output</label>

            <br>
            <div class="buttons-flex">
                <!-- Cancel -->
                <button class="bag-buttons" on:click={() => {isAccordionOpen = false; isBagManagerOpen = true; isCloneMenuOpen = false; }}>Cancel</button>

                <!-- Copy command -->
                <button class="bag-buttons" on:click={() => {isCloneMenuOpen = true; 
                    vscode.postMessage({type: "cloneConfirm", values: {
                        newBagPath: cloneBagPath + "/" +(cloneName.endsWith(".bag") ? cloneName : cloneName + ".bag"),
                        startTime: {sec: Math.trunc(range[0]), nsec: range[0] - Math.trunc(range[0])},
                        endTime: {sec: Math.trunc(range[1]), nsec: range[1] - Math.trunc(range[1])},
                        verbose: verbose,
                        topics: selectedTopic,
                        copy: true
                    }})}}>Copy Command</button>

                <!-- Clone confirm -->
                <button class="bag-buttons" on:click={() => {isCloneMenuOpen = true; 
                vscode.postMessage({type: "cloneConfirm", values: {
                    newBagPath: cloneBagPath + "/" +(cloneName.endsWith(".bag") ? cloneName : cloneName + ".bag"),
                    startTime: {sec: Math.trunc(range[0]), nsec: range[0] - Math.trunc(range[0])},
                    endTime: {sec: Math.trunc(range[1]), nsec: range[1] - Math.trunc(range[1])},
                    verbose: verbose,
                    topics: selectedTopic,
                    copy: false
                }})}}>Clone</button>
            </div>
        {/if}
    {/if}
{:else}

    <!--
        recordBag { 
            topics: [],
            recordAll: false,
            regex: null,
            regexExclude: null,
            quiet: true,
            duration: null,
            prefix: null,
            name: null,
            split: null,
            maxSplits: null,
            bufferSize: null,
            node: null
        }
    } -->

    <!-- Name bag -->
    <label for="bag-name">Bag prefix</label>
    <input bind:value={recordBag.name} disabled={isRecording}>
    <br>

    <!-- Bag location -->
    <label for="bag_location"><b>New bag location:</b></label>
    <input id="bag_location" type="text" class="margin-top-5 location-input" bind:value={recordBag.location} style="border:solid 1px black; width:88%">
    <button class="location-btn" on:click={() => {vscode.postMessage({type: 'setRecordPath', value:recordBag.location})}}>...</button>
    <br>

    <!-- Record Topics -->
    <label for="record-all">Record all topics</label>
    <input type=checkbox bind:checked={recordBag.recordAll} name="record-all" disabled={isRecording}>

    <button class="accordion-button" on:click={toggle} aria-expanded={isAccordionOpen}><svg style="tran"  width="20" height="20" fill="none" stroke-linecap="round" stroke-linejoin="round" stroke-width="2" viewBox="0 0 30 10" stroke="currentColor"><path d="M9 5l7 7-7 7"></path></svg><b>Active Topics</b></button>
    {#if isAccordionOpen}
        <ul><button class="bag-buttons" on:click={() => {topics.length === recordBag.topics.length ? recordBag.topics = [] : recordBag.topics = topics}} disabled={isRecording}>Select all</button></ul>
        <ul style="list-style: none" transition:slide={{ duration: 300 }}>
            {#each topics as item}
                <li><label><input type=checkbox bind:group={recordBag.topics} value={item} disabled={isRecording || recordBag.recordAll}>{item}</label></li>
            {/each}
        </ul>
    {/if}

    <label for="quiet">Quiet Output</label>
    <input type=checkbox bind:checked={recordBag.quiet} name="quiet" disabled={isRecording}>

    <div class="buttons-flex">

    <br>
    <button class="bag-buttons" on:click={() => {recordMenuOpen = false; isBagManagerOpen = false;}} disabled={isRecording}>Cancel</button>

    <button class="bag-buttons" on:click={() => {vscode.postMessage({type: "recordBag", values: {recordBag: recordBag, copy: true}})}}>Copy Command</button>

    <!-- Start recording -->
    <!-- TODO: Error checking and disable button on error? -->
    <button class="bag-buttons" on:click={() => {
        if (!isRecording) {
            vscode.postMessage({type: "recordBag", values:{recordBag: recordBag, copy: false}})
        } else {
            vscode.postMessage({type: "stopRecording"});
        }

        isRecording = !isRecording;
        
        }} disabled={((!recordBag.recordAll && recordBag.topics.length === 0) || (recordBag.location === ""))}>{isRecording ? "Stop Recording" : "Record"}</button>

    </div>
    
    
{/if}
