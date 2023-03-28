<script>
    const vscode = acquireVsCodeApi();
    
    let isRvizOpen = false;
    let isNodeGraphOpen = false;
        
</script>

<h3 style="text-align:center;margin: 10px 0px;">Visuals (Coming soon...)</h3>
<hr style="margin-bottom:10px">

{#if !isRvizOpen}
    <!--<button on:click={() => {vscode.postMessage({type: 'webviewRviz'})}}>Rviz</button>-->
    <button disabled={true}>Rviz</button>
    {/if}
    
{#if !isNodeGraphOpen}
    <!--<button on:click={() => {vscode.postMessage({type: 'webviewNodeGraph'})}}>Node Graph</button>-->
    <button disabled={true}>Node Graph</button>
    {/if}
    
          
            
        
       
                
            

        
                



