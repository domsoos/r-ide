<!--
<script>
    export let nodes;

</script>
    
<ul>
    {#each nodes as item, i}
    <li>
        {#if item.children}
             svelte-ignore a11y-click-events-have-key-events
            <span on:click={() => {item.expanded = !item.expanded}}>
                <span class="arrow {item.expanded ? 'arrowDown': ''}">&#x25b6</span>
                {item.property} {item.type}
            </span>
            {#if item.expanded}
                <svelte:self nodes={item.children}/>
            {/if}
        {:else}
            <div style="display: flex; justify-content: space-between;">
                <span>
                    <span class="no-arrow"/>
                        {item.property} {item.type}: {item.value}
                </span>
            </div>

        {/if}
    </li>
    {/each}
</ul>
    
<style>
    ul {
        margin: 0;
        list-style: none;
        padding-left: 1.2rem; 
        user-select: none;
    }
    .no-arrow { padding-left: 1.0rem; }
    .arrow {
        cursor: pointer;
        display: inline-block;
        /* transition: transform 200ms; */
    }
    .arrowDown { transform: rotate(90deg); }
</style>


-->


<script>
    export let nodes;

  function renderTreeNode(node) {
    if (node.children) {
      return `
        <li>
          <span><b style="margin-right:10px">${node.property}</b> <span style="float:right;font-size:12px"><i>[${node.type}]</i></span></span>
          <ul>${node.children.map(renderTreeNode).join('')}</ul>
        </li>
      `;
    } else {
      return `<li><span><b style="margin-right:10px">${node.property}:</b> ${node.value} <span style="float:right;font-size:12px"><i>[${node.type}]</i></span></span></li>`;
    }
  }
</script>

<ul>{#each nodes as node}{@html renderTreeNode(node)}{/each}</ul>


