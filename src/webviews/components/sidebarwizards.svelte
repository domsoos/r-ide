<script>
	const vscode = acquireVsCodeApi();
	let workspaceDirectory;

	vscode.postMessage({
                type: 'getWorkspace'
    });

	window.addEventListener('message', event => {
		const message = event.data; // The JSON data our extension sent
		switch (message.type) {
			case 'setWorkspace':
				workspaceDirectory = message.value;
				break;
		}
	});

	let isMenuOpen = false;
	let isWizardOpen = false;
</script>

{#if !isWizardOpen}
	<div class="dropdown">
		<button on:click={() =>{isMenuOpen = !isMenuOpen}}>Create New</button>
		{#if isMenuOpen}
			<div class="dropdown-content">
				<button on:click={() =>{isWizardOpen = true; isMenuOpen = false;}}>ROS Node</button>
				<button on:click={() =>{isWizardOpen = true; isMenuOpen = false;}}>ROS Msg</button>
				<button on:click={() =>{isWizardOpen = true; isMenuOpen = false;}}>ROS Srv</button>
			</div>
		{/if}
	</div>
{/if}

{#if isWizardOpen}
	<div class="wizard-container">
		<h3 style="text-align: center;" id="wizard-title">Creation Wizard</h3>
		<hr>
		<label for="wizard-file-type">File type:</label>
		<br>
		<select name="wizard-file-type" id="wizard-file-type" class="width-100 margin-top-5">
			<option value="cplusplus">C++</option>
			<option value="python">Python</option>
		</select>
		<br>
		<br>
		<label for="wizard-node-name">Node name:</label>
		<input type="text" id="wizard-node-name" class="margin-top-5" style="border:solid 1px black">
		<br>
		<label for="wizard-node-location">Node location:</label>
		<input type="text" id="wizard-node-location" class="margin-top-5" value="{workspaceDirectory}" style="border:solid 1px black">
		<br>
		<input type="checkbox" name="publisher" id="publisher">
		<label for="wizard-node-publisher">Publisher</label>
		<br>
		<input type="checkbox" name="subscriber" id="subscriber" class="margin-top-5">
		<label for="wizard-node-publisher">Subscriber</label>
		<br>
		<br>
		<button class="cancel-btn" id="cancel-btn" on:click={() =>{isWizardOpen = false}}>Cancel</button>
		<button class="next-btn" id="next-btn" on:click={() =>{isWizardOpen = false}}>Next</button>
	</div>
{/if}