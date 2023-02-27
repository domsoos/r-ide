<script>
    export let topics
    export let updateCheckboxes;
    export let vscode;
    export let onExpand;

    const handleChange = (event, topic) => {
        topic.checked = event.target.checked;
        updateCheckboxes(topic);
    };

    function onExpandChange(item){
        onExpand(item);
    }

</script>
    
<ul>
    {#each topics as item, i}
    <li>
        {#if item.children.length > 0}
            <!-- svelte-ignore a11y-click-events-have-key-events -->
            <span on:click={() => {item.expanded = !item.expanded;onExpandChange(item)}}>
                <span class="arrow {item.expanded ? 'arrowDown': ''}">&#x25b6</span>
                {item.topic}
            </span>
            {#if item.expanded}
                <svelte:self topics={item.children}  updateCheckboxes={updateCheckboxes} vscode={vscode} onExpand={onExpandChange}/>
            {/if}
        {:else}
            <div style="display: flex; justify-content: space-between;">
                
                <span>
                    <span class="no-arrow"/>
                    <label for="{item.fulltopic}-checkbox">
                        {item.topic}
                    </label>
                </span>
            
                <input type="checkbox" id="{item.fulltopic}-checkbox" bind:checked={item.checked}  on:change={(e) => handleChange(e, item)}/>
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