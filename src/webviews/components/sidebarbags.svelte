<script>
    const vscode = acquireVsCodeApi();
    import Slider from '@bulatdashiev/svelte-slider';
    import { slide } from "svelte/transition";

    /* Accordion Code */
    let isAccordionOpen = false
	const toggle = () => isAccordionOpen = !isAccordionOpen
    /* Accordion Code END*/

    let topics = [];
    let messages = [];
    let isPlaying = false;


    let isBagManagerOpen = false;
    let isRecording = false;
    let selectedBag = null;
    let selectedBagPath = null;
    let cloneBagPath = null;
    let isCloneMenuOpen = false;
    let messagesLoaded = false;
    let connectionsLoaded = false;
    
    let range = [0,1]; 

    window.addEventListener('message', event => {
        // console.log(event);
		const message = event.data; // The JSON data our extension sent
		switch (message.type) {
			case 'setSelectedBag':{
                // TODO: Possible race condition on extremely small bags?
                connectionsLoaded = false;
                messagesLoaded = false;

                selectedBagPath = message.value.path;                
                selectedBag = selectedBagPath.substring(selectedBagPath.lastIndexOf('/'));
                cloneBagPath = selectedBagPath.substring(0, selectedBagPath.lastIndexOf('/'));
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
                    console.log(messages.length);
                } else {
                    messages.push(...message.value);
                    console.log(messages.length);
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
            }
		}
	});

</script>

<h3 style="text-align:center;margin: 10px 0px;">ROS Bag Manager</h3>
<hr style="margin-bottom:10px">
<!-- Recording a new bag UNIMPLEMENTED -->
{#if !isBagManagerOpen}
    <button disabled='{isRecording}' on:click={() => {isBagManagerOpen = true;}}>Manage Bags</button>
    {#if !isRecording}
        <button on:click={() => {isRecording = true;}}>Start Recording</button>
    {:else}
        <button on:click={() => {isRecording = false;}}>Stop Recording</button>
    {/if}

<!-- Managing existing bags -->
{:else}
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
        {#if !isCloneMenuOpen}
            <div class="buttons-flex">
                <button class="bag-buttons" on:click={() => {isBagManagerOpen = true;}}>Cancel</button>
                <button class="bag-buttons" on:click={() => {isCloneMenuOpen = true;}}>Clone</button>
                {#if !isPlaying}
                    <button class="bag-buttons" on:click={() => {vscode.postMessage({type: 'playBag'}); isPlaying = true}} disabled={!(connectionsLoaded && messagesLoaded)}>Play</button>
                {:else}
                    <button class="bag-buttons" on:click={() => {vscode.postMessage({type: 'pauseBag'}); isPlaying = false;}}>Pause</button>               
                {/if}
            </div>

        <!-- Clone bag menu -->
        {:else}
            <br>
            <br>
            <label for="bag_name"><b>New bag name:</b></label>
            <input id="bag_name" type="text" class="margin-top-5" style="border:solid 1px black" placeholder="Enter new bag name...">
            <br>
            <label for="bag_location"><b>New bag location:</b></label>
            <input id="bag_location" type="text" class="margin-top-5 location-input" value="{cloneBagPath? cloneBagPath : "TODO::NEED TO FIX THIS"}" style="border:solid 1px black; width:88%">
            <button class="location-btn" on:click={() => {vscode.postMessage({type: 'getCloneBagPath', value:cloneBagPath ? cloneBagPath : selectedBagPath})}}>...</button>
            <br>
            <br>
            <b>Trim clone:</b>
            <div class="buttons-flex">
                <input type="number" bind:value={range[0]} style="width: 50px">
                <input type="number" bind:value={range[1]} style="width: 50px">
            </div> 
            <Slider max="100" step="1" bind:value={range} range order style="margin-right:20px"/>

            <button class="accordion-button" on:click={toggle} aria-expanded={isAccordionOpen}><svg style="tran"  width="20" height="20" fill="none" stroke-linecap="round" stroke-linejoin="round" stroke-width="2" viewBox="0 0 30 10" stroke="currentColor"><path d="M9 5l7 7-7 7"></path></svg><b>Filter Topics</b></button>
            {#if isAccordionOpen}
                <ul style="list-style: none" transition:slide={{ duration: 300 }}>
                    {#each topics as item}
                        <li><input type=checkbox>{item}</li>
                    {/each}
                </ul>
            {/if}

            <br>
            <div class="buttons-flex">
                <button class="bag-buttons" on:click={() => {isAccordionOpen = false; isBagManagerOpen = true; isCloneMenuOpen = false}}>Cancel</button>
                <button class="bag-buttons" on:click={() => {isCloneMenuOpen = true;}}>Clone</button>
            </div>
        {/if}
    {/if}
{/if}



