<script>
	const vscode = acquireVsCodeApi();

	let isMenuOpen = false;
	let isWizardOpen = false;

	let workspaceDirectory;

	let nodeName = '';
	let fileLocation = '';
	console.log(fileLocation);
	let nodeLanguage = {};
	let isPublisher = false;
	let isSubscriber = false;
	let options = [
		{id: 1, value: 'cpp', text:'C++'},
		{id: 2, value: 'py', text: 'Python'}
	]

	vscode.postMessage({
                type: 'getWorkspace'
    });

	window.addEventListener('message', event => {
		const message = event.data; // The JSON data our extension sent
		switch (message.type) {
			case 'setWorkspace':
				workspaceDirectory = message.value;
				fileLocation = message.value;
				break;
		}
	});



	function nextButtonFunction() {
		// TODO: Currently overwrites without warning, should ask for confirmation for overwrites

		// TODO: Currently isSubscriber and isPublisher do nothing

        if (nodeName === '') {
            // No given name
            vscode.postMessage({
                type: 'onError',
                value: 'The node must have a name'
            });
        } else {
            // Success!
			console.log(nodeLanguage);
            vscode.postMessage({
                type: 'r-ide.command',
                value: {
                    command: 'r-ide.create-file-from-template',
                    args: [fileLocation + '/' + nodeName + '.' + nodeLanguage.value, new TextEncoder().encode('placeholder text')]
                }
            });
        }
	}
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
		<!-- Title -->
		<h3 style="text-align: center;" id="wizard-title">Creation Wizard</h3>
		<hr>
		<label for="wizard-file-type">File type:</label>
		<br>

		<!-- File type -->
		<select name="wizard-file-type" id="wizard-file-type" class="width-100 margin-top-5" bind:value={nodeLanguage}>
			{#each options as option}
				<option value={option}>
					{option.text}
				</option>
			{/each}
		</select>
		<br>
		<br>

		<!-- Node Name -->
		<label for="wizard-node-name">Node name:</label>
		<input type="text" id="wizard-node-name" class="margin-top-5" bind:value={nodeName} style="border:solid 1px black">
		<br>

		<!-- File Location -->
		<label for="wizard-node-location">Node location:</label>
		<input type="text" id="wizard-node-location" class="margin-top-5" bind:value={fileLocation} style="border:solid 1px black">
		<br>

		<!-- Is Publisher -->
		<input type="checkbox" name="publisher" id="publisher" bind:value={isPublisher}>
		<label for="wizard-node-publisher">Publisher</label>
		<br>

		<!-- Is subscriber -->
		<input type="checkbox" name="subscriber" id="subscriber" class="margin-top-5" bind:value={isSubscriber}>
		<label for="wizard-node-subscriber">Subscriber</label>
		<br>
		<br>

		<!-- Cancel and Next buttons -->
		<button class="cancel-btn" id="cancel-btn" on:click={() =>{isWizardOpen = false;}}>Cancel</button>
		<button class="next-btn" id="next-btn" on:click={() => {isWizardOpen = false; nextButtonFunction();}}>Next</button>
	</div>
{/if}