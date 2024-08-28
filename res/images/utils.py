import os
from PIL import Image

def eps2pdf(eps_path, pdf_path):
    # Convert EPS to PDF using PIL
    with Image.open(eps_path) as img:
        img.save(pdf_path, 'PDF')

def main():
    directory = os.path.dirname(__file__)
    eps_filename = 'PathBeforeTrajectory.eps'
    pdf_filename = eps_filename.replace('.eps', '.pdf')
    png_filename = eps_filename.replace('.eps', '.png')

    eps_path = os.path.join(directory, eps_filename)
    pdf_path = os.path.join(directory, pdf_filename)
    png_path = os.path.join(directory, png_filename)

    eps2pdf(eps_path, pdf_path)
    # pdf2png(pdf_path, png_path)

    print(f"Converted {eps_path} to {pdf_path}")

if __name__ == "__main__":
    main()


