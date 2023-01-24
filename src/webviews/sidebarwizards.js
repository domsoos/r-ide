(function () {
    const vscode = acquireVsCodeApi();

    const createNewDropdown = document.getElementById('dropdown-content');
    const createNewButton = document.getElementById('create-new-btn');
    const wizardContainer = document.getElementById('wizard-container');
    const rosNodeButton = document.getElementById('ros-node-btn');
    const rosMsgButton = document.getElementById('ros-msg-btn');
    const rosSrvButton = document.getElementById('ros-srv-btn');
    const cancelButton = document.getElementById('cancel-btn');
    const nextButton = document.getElementById('next-btn');
    const wizardTitle = document.getElementById('wizard-title');
    // const fileLocation = document.getElementById('wizard-node-location').value;
    // const nodeName = document.getElementById('wizard-node-name');
    // const nodeLanguage = document.getElementById('wizard-file-type');
    // const fileURI = Uri.file(fileLocation + nodeName + '.' + nodeLanguage);

    let isDropdownOpen = false;
    let isWizardOpen = false;


    rosNodeButton.addEventListener('click', () =>{
        createNewButton.style.display = 'none';
        wizardTitle.innerText = 'ROS Node Wizard';
        wizardContainer.style.display = 'block';
        isWizardOpen = true;
    });

    rosMsgButton.addEventListener('click', () =>{
        createNewButton.style.display = 'none';
        wizardTitle.innerText = 'ROS Msg Wizard';
        wizardContainer.style.display = 'block';
        isWizardOpen = true;
    });

    rosSrvButton.addEventListener('click', () =>{
        createNewButton.style.display = 'none';
        wizardTitle.innerText = 'ROS Srv Wizard';
        wizardContainer.style.display = 'block';
        isWizardOpen = true;
    });

    cancelButton.addEventListener('click', () =>{
        createNewButton.style.display = 'block';
        wizardTitle.innerText = 'Creation Wizard';
        wizardContainer.style.display = 'none';
        isWizardOpen = false;
    });

    nextButton.addEventListener('click', () => {
        createNewButton.style.display = 'block';
        wizardTitle.innerText = 'Creation Wizard';
        wizardContainer.style.display = 'none';
        isWizardOpen = false;
        // TODO BUG: Non functional command, find where to fix
        vscode.executeCommand("r-ide.create-file-from-template");
    });


    window.onclick = function(event) {
        if(event.target === createNewButton && !isDropdownOpen){
            createNewDropdown.style.display = 'block';
            isDropdownOpen = true;
        }else if(event.target === createNewButton && isDropdownOpen){
            createNewDropdown.style.display = 'none';
            isDropdownOpen = false;
        }

        if (event.target !== createNewButton && isDropdownOpen) {
            createNewDropdown.style.display = "none";
            isDropdownOpen = false;
        }
    };


}());