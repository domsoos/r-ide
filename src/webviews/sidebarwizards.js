(function () {
    const vscode = acquireVsCodeApi();

    const createNewDropdown = document.getElementById('dropdown-content');
    const createNewButton = document.getElementById('create-new-btn');
    let isDropdownOpen = false;

    /*
    createNewButton.addEventListener('click', () =>{
        createNewDropdown.style.display = 'block';
        isDropdownOpen = true;
    });
    */

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