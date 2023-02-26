<script>
    const vscode = acquireVsCodeApi();

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
