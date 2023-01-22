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