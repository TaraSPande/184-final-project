## Frontend Explaination 

You can access the front end at: [taraspande.github.io/184-final-project/](https://taraspande.github.io/184-final-project/)
- This page is the index.html file

To access the alternative pages, you need to add "tests/[name of the html]/" at the end of the URL:
- For example: access milestone.html at [taraspande.github.io/184-final-project/tests/milestone/](https://taraspande.github.io/184-final-project/tests/milestone/)

How to debug:
- Wait for your code to push to github and redeploy. On the frontend, hit Command+Option+I (for Mac) to inspect the page. Go to the console tab and you can see what the errors are.
     - IGNORE: "ColladaLoader.js:3408 THREE.ColladaLoader: There is already a node with ID null. Exclude current node from further processing..."
     - IGNORE: "Failed to load resource: the server responded with a status of 404 ()"

## How it Works

### Frontend Pages
- **index.html** --> This is where our final frontend should go! Currently, its the progress made since the milestone (don't mess with this for now)
- **tests/milestone.html** --> Frontend used in the milestone assignment.
- **tests/translate.html** --> applyTransformation (from Tamnhi) baked into the code directly.
- **tests/translate-gui.html** --> applyTransformation (from Tamnhi) takes in inputs from the GUI to translate/rotate the object.
- **tests/webgl_geometry_teapot.html** --> what I based the frontend off of originally (stolen from [https://threejs.org/examples/#webgl_geometry_teapot](https://threejs.org/examples/#webgl_geometry_teapot))

### Backend Files
- **backend** --> where the JavaScript versions of the C files are!
- **build** and **jsm** and **textures** --> this is the main backend (don't mess with this at all)
- **models** --> you can add .dae files here to load into the frontend!
