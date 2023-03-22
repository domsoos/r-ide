<script>
    const vscode = acquireVsCodeApi();
    import Slider from '@bulatdashiev/svelte-slider';
    import { slide } from "svelte/transition";
    import ROSLIB from 'roslib';

    /* Accordion Code */
    let isAccordionOpen = false
	const toggle = () => isAccordionOpen = !isAccordionOpen
    /* Accordion Code END*/

    let topics = [];
    let messages = [];
    let publishers = new Map();
    let isPlaying = false;


    let isBagManagerOpen = false;
    let isRecording = false;
    let selectedBag = null;
    let selectedBagPath = null;
    let cloneBagPath = null;
    let isCloneMenuOpen = false;
    let isConnected;

    let rosApi;
    
    let range = [0,1]; 

    // onMount(async () => {
    //     try {
    //         await new ROS();
    //         rosApi = ROS.getROSApi();
    //     } catch (err) {
    //         isConnected = false;
    //         console.error(err);
    //         vscode.postMessage({
    //             type: 'r-ide.noConnection',
    //         });
    //     }
    // });

    function waitForCondition(leadup) {
        console.log(leadup);
        return new Promise((resolve) => {
            setTimeout(() => {resolve();}, leadup);
        });
    }

    async function playBag(startTime) {
        isPlaying = true;
        let {i, leadup} = findFirstMessage(startTime);

        console.log(messages.length);
        while (i < messages.length - 1) {
            const {message, topic, timestamp} = messages[i];

            await waitForCondition(leadup);

            publishers.get(topic).publish(messages[i]);
            // console.log(message);

            i++;

            // fast convert seconds and nanoseconds into milliseconds
            // console.log(messages[i]);
            const nextTimeStamp = messages[i].timestamp;
            leadup = ((nextTimeStamp.sec - timestamp.sec) * 1000) + ((nextTimeStamp.nsec >> 20) - (timestamp.nsec >> 20));
        }

        console.log('finished playing');
        
    }

    async function setupPublishers(connections) {
        topics = []
        publishers = new Map();
        for (let conn of connections) {
            topics.push(conn.topic);
            let newPublisher = new ROSLIB.Topic({
                ros: rosApi,
                messageType: conn.type,
                name: conn.topic
            });

            newPublisher.advertise();

            publishers.set(conn.topic, newPublisher);

        }

        console.log([...publishers.values()])
    }

    /**
     * Returns the first index of the first message to play and the wait time to play it
     * @param startTime The time from the first message that the bag starts. Starts with the first message if startTime === 0
     */
    function findFirstMessage(startTime) {
        if (startTime === 0) {
            return {i: 0, leadup: 0};

        // After last message
        } else if (false) {
            return messages.length;
        }

        // TODO: Binary search to find first message
        return {i: 0, leadup: 0};
    }

    window.addEventListener('message', event => {
        // console.log(event);
		const message = event.data; // The JSON data our extension sent
		switch (message.type) {
			case 'setSelectedBag':{
                messages = [];
                topics = [];

                
                for (let p of publishers.keys()) {
                    publishers.get(p).unadvertise();
                }
                
                publishers.clear();

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
                <button class="bag-buttons" on:click={() => {isBagManagerOpen = true}}>Cancel</button>
                <button class="bag-buttons" on:click={() => {isCloneMenuOpen = true;}}>Clone</button>
                <button class="bag-buttons" on:click={() => {vscode.postMessage({type: 'playBag'})}}>Play</button>
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



