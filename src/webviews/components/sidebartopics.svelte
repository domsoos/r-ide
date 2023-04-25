<script>
    const vscode = acquireVsCodeApi();

    function openTopicMonitor(){
        vscode.postMessage({
            type: 'openTopicMonitor',
        });
    };

    let isDebugging = false;
    let rosPackage = '';
    let rosNode = '';

    function debugRosNode(){
        vscode.postMessage({
            type: 'debugRosNode',
            value: {
                rosPackage,
                rosNode
            }
        });

        isDebugging = false;
    }

    function killNode(){
        vscode.postMessage({
            type: 'killNode',
        });
    }

    function srcCatkinSetup(){
        vscode.postMessage({
            type: 'srcCatkinSetup',
        });
    }
</script>

<button style="margin-top:5px;" on:click={() => {openTopicMonitor()}}>ROS Topic Monitor</button>
<button style="margin-top:5px;" on:click={() => {srcCatkinSetup()}}>Source Catkin Setup.bash</button>

<!-- Commented out Debug ROS node and Topic publisher button for Beta 

{#if !isDebugging}
    <button on:click={() => {openTopicMonitor()}}>ROS Topic Monitor</button>
    <button>ROS Topic Publisher</button>
    <button on:click={() => {isDebugging = true;}}>Debug ROS Node</button>
    <button on:click={() => {killNode()}}>Kill Node</button>
{:else}
    <label for="node-package">Package Name:</label>
    <input type="text" id="node-package" bind:value={rosPackage} style="border:solid 1px black">
    <label for="node-name">Node Name:</label>
    <input type="text" id="node-name" bind:value={rosNode} style="border:solid 1px black">
    <button on:click={() => {debugRosNode()}}>Debug Node</button>
{/if}
-->


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
