# Converting Code Notebooks to PDF

## Using Colab
  - If you are running your notebook in Colab:
    - Go to `File > Print`.
    - Download the notebook as a PDF from your browser.

## Using a Notebook in a Text Editor (e.g., VSCode)
  1. Make sure you have installed `nbconvert` by running:
     - `pip install nbconvert`
  2. Convert your notebook to HTML:
     - `jupyter nbconvert --to html <path to notebook>`
  3. Open the generated `.html` file in your browser.
  4. Go to `File > Print` in your browser to download the notebook as a PDF.
     - (Tip: Adjust the `More settings > Scale` option as needed to ensure all cell content is visible.)