<script>
	const vscode = acquireVsCodeApi();
	import { onMount } from 'svelte';

	let isMenuOpen = false;
	let isWizardOpen = false;

	let workspaceDirectory;
	
	let isPublisher = false;
	let isSubscriber = false;

	let packageName = null;

	onMount(async () => {
		window.addEventListener('message', event => {
		const message = event.data; // The JSON data our extension sent
			switch (message.type) {
				case 'setWorkspace': {
					const { name, path } = message.value;
					workspaceDirectory = path;
					packageName = name;
					break;
				}
			}
		});

		// Get Workspace Directory
		vscode.postMessage({ 
			type: 'getWorkspace' 
		});
		
 	});




	let nodeName = '';
	let wizardTitle = 'Creation Wizard';

	let languages = {
		"Node": [
			{id: 'cpp', text: 'C++' , value: ".cpp", dir: "/src/"},
			{id: 'py', text: 'Python', value: ".py", dir: "/scripts/"}],

		"Srv": [
			{id: 'srv', text: 'Service File', value: ".srv", dir: "/srv/"}
		],

		"Msg": [
			{id: 'msg', text: 'Message File', value: ".msg", dir: "/msg/"}
		]
	};

	let selectedLanguage = languages[0];
	let selectedWizardType = '';
	let isNameValid = true;
	let invalidNameMessage = '';
	let isPackageSelected = true;

	function validateFileName(){
		const extRegex = /^.*\.[^.]+$/;
		if(nodeName.trim() === ''){
			invalidNameMessage = 'Please specify a name';
			return false;
		}
		else if(extRegex.test(nodeName)){
			invalidNameMessage = 'Name must not include a file extension';
			return false;
		}
		else{
			invalidNameMessage = '';
			return true;
		}
	}

	function nextButtonClicked(){
		isNameValid = true;
		isPackageSelected = true;
		// TODO: Currently overwrites without warning, should ask for confirmation for overwrites
		console.log(packageName + " Here")
		if (!validateFileName() || !packageName) {
			if(!validateFileName() && !packageName){
				isNameValid = false;
				isPackageSelected = false;
			}
			else if(!validateFileName()){
				isNameValid = false;
			}
			else{
				isPackageSelected = false;
			}
		} else {
			// Success!
			wizardTitle = 'Creation Wizard';
			isWizardOpen = false;

			switch (selectedWizardType) {
				case ("Node"): {
					vscode.postMessage({
						type: 'r-ide.command',
						value: {
							command: 'r-ide.create-file-from-template',
							args: [
								workspaceDirectory + selectedLanguage.dir + '/' + nodeName + selectedLanguage.value, 
								{
									language: selectedLanguage,
									isPublisher: isPublisher,
									isSubscriber: isSubscriber,
								}
							]
						}
					});
					break;
				}

				case ("Msg"): {
					vscode.postMessage({
						type: 'r-ide.command',
						value: {
							command: 'r-ide.create-msg',
							args: [
								workspaceDirectory + selectedLanguage.dir + '/' + nodeName + selectedLanguage.value
							]
						}
					});
					break;
				}

				case ("Srv"): {
					vscode.postMessage({
						type: 'r-ide.command',
						value: {
							command: 'r-ide.create-srv',
							args: [
								workspaceDirectory + selectedLanguage.dir + '/' + nodeName + selectedLanguage.value
							]
						}
					});
					break;
				}
			};

			vscode.postMessage({
				type: 'addEventToDB',
				value: selectedWizardType + " creation wizard used"
			});
			selectedLanguage = languages[selectedWizardType][0];
		}
	}

	function wizardSelected(wizardType){
		resetWizardFields();
		selectedWizardType = wizardType;
		isWizardOpen = true; 
		isMenuOpen = false; 
		wizardTitle = 'ROS ' + wizardType + ' Wizard';
	}

	function resetWizardFields(){
		selectedLanguage = languages[0];
		selectedWizardType = '';
		isNameValid = true;
		invalidNameMessage = '';
		isPackageSelected = true;
		nodeName = '';
		wizardTitle = 'Creation Wizard';
		isPublisher = false;
		isSubscriber = false;
		packageName = null;
	}

</script>



	{#if !isWizardOpen}
		<h3 style="text-align:center;margin: 10px 0px;">ROS Creation Wizard</h3>
		<hr style="margin-bottom:10px">
		<div class="dropdown">
			<button on:click={() =>{isMenuOpen = !isMenuOpen}}>Create New</button>
			{#if isMenuOpen}
				<div class="dropdown-content">
					<button on:click={() =>{wizardSelected("Node")}}>ROS Node</button>
					<button on:click={() =>{wizardSelected("Msg")}}>ROS Msg</button>
					<button on:click={() =>{wizardSelected("Srv")}}>ROS Srv</button>
				</div>
			{/if}
		</div>
	{/if}

	{#if isWizardOpen}
		<div class="wizard-container">
			<!-- Title -->
			<h3 style="text-align: center;">{wizardTitle}</h3>
			<hr>
			<label for="wizard-file-type">File type:</label>
			<br>

			<!-- File type -->
			<select name="wizard-file-type" class="width-100 margin-top-5" bind:value={selectedLanguage}>
				{#each languages[selectedWizardType] as language}
					<option value="{language}">{language.text}</option>
				{/each}
			</select>
			<br>
			<br>

			<!-- Node Name -->	
			<label for="wizard-node-name">{selectedWizardType} name:</label>
			<input type="text" bind:value={nodeName} class="margin-top-5" style="border:solid 1px black">
			{#if !isNameValid}
				<p style="color: #ff4444; font-size: 12px;">{invalidNameMessage}</p>
			{/if}
			<br>

			<!-- File Location -->
			<label for="wizard-node-location">{selectedWizardType} location:</label>
			<input type="text" readonly class="margin-top-5 location-input" value="{packageName ? packageName : "Select a package..."}" style="border:solid 1px black; width:88%">
			<button class="location-btn" on:click={() => {vscode.postMessage({type: 'openFileExplorer', value:workspaceDirectory})}}>...</button>
			{#if !isPackageSelected}
				<p style="color: #ff4444; font-size: 12px;">Please select a package</p>
			{/if}
			<br>

			{#if selectedWizardType === 'Node'}
				<!-- Is Publisher -->
				<input type="checkbox" name="publisher" id="wizard-node-publisher" bind:checked={isPublisher}>
				<label for="wizard-node-publisher">Publisher</label>
				<br>

				<!-- Is subscriber -->
				<input type="checkbox" name="subscriber" id="wizard-node-subscriber" class="margin-top-5" bind:checked={isSubscriber}>
				<label for="wizard-node-subscriber">Subscriber</label>
				<br>
				<br>
			{/if}
			
			
			<!-- Cancel and Next buttons -->
			<button class="cancel-btn" on:click={() =>{isWizardOpen = false; wizardTitle = 'Creation Wizard'}}>Cancel</button>
			<button class="next-btn" on:click={nextButtonClicked}>Next</button>
		</div>
	{/if}
