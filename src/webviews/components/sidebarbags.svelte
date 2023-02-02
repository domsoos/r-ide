<script>
    const vscode = acquireVsCodeApi();
    import Slider from '@bulatdashiev/svelte-slider';
    import { slide } from "svelte/transition";

    /* Accordion Code */
    let isOpen = false
	const toggle = () => isOpen = !isOpen
    let topics = [{value:"geometry_msgs/Twist"},{value:"geometry_msgs/PoseStamped"},{value:"rosgraph_msgs/Log"},{value:"turtlesim/Pose"}];
    /* Accordion Code END*/


    let isBagManagerOpen = false;
    let isRecording = false;
    let selectedBag = null;
    let selectedBagPath = null;
    let cloneBagPath = null;
    let isCloneMenuOpen = false;
    
    let range = [0,100]; 

    window.addEventListener('message', event => {
		const message = event.data; // The JSON data our extension sent
		switch (message.type) {
			case 'setSelectedBag':{
                selectedBagPath = message.value;
                // TODO: Regex for forward and backslash
                //let regex = new RegExp("/\\|\//");
                let pathToArr = selectedBagPath.split("/");
                selectedBag = pathToArr[pathToArr.length - 1];
				break;
            }
            case 'setCloneBagPath':{
                cloneBagPath = message.value;
				break;
            }
		}
	});

</script>

<h3 style="text-align:center;margin: 10px 0px;">ROS Bag Manager</h3>
<hr style="margin-bottom:10px">
{#if !isBagManagerOpen}
    <button disabled='{isRecording}' on:click={() => {isBagManagerOpen = true;}}>Manage Bags</button>
    {#if !isRecording}
        <button on:click={() => {isRecording = true;}}>Start Recording</button>
    {:else}
        <button on:click={() => {isRecording = false;}}>Stop Recording</button>
    {/if}
{:else}
    <button on:click={() => {vscode.postMessage({type: 'getSelectedBag'})}}>{selectedBag == null? 'Select Bag..': 'Selected Bag: .../' + selectedBag}</button>
    {#if selectedBag === null}
        <button on:click={() => {isBagManagerOpen = false}}>Cancel</button>
    {/if}
    {#if selectedBag !== null}
        <!--<div style="margin:5px 0px;">Selected Bag: .../{selectedBag}</div>-->
        {#if !isCloneMenuOpen}
            <div class="buttons-flex">
                <button class="bag-buttons" on:click={() => {selectedBag = null; isBagManagerOpen = false}}>Cancel</button>
                <button class="bag-buttons" on:click={() => {isCloneMenuOpen = true;}}>Clone</button>
                <button class="bag-buttons">Play</button>
            </div>
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
            <b>Trim from:</b> {range[0]} - {range[1]}
            <Slider max="100" step="1" bind:value={range} range order style="margin-right:20px"/>

            <button class="accordion-button" on:click={toggle} aria-expanded={isOpen}><svg style="tran"  width="20" height="20" fill="none" stroke-linecap="round" stroke-linejoin="round" stroke-width="2" viewBox="0 0 30 10" stroke="currentColor"><path d="M9 5l7 7-7 7"></path></svg><b>Filter Topics</b></button>
            {#if isOpen}
                <ul style="list-style: none" transition:slide={{ duration: 300 }}>
                    {#each topics as item}
                        <li><input type=checkbox>{item.value}</li>
                    {/each}
                </ul>
            {/if}

            <br>
            <div class="buttons-flex">
                <button class="bag-buttons" on:click={() => {isOpen = false;selectedBag = null; isBagManagerOpen = false; isCloneMenuOpen = false}}>Cancel</button>
                <button class="bag-buttons" on:click={() => {isCloneMenuOpen = true;}}>Clone</button>
            </div>
        {/if}
    {/if}
{/if}



