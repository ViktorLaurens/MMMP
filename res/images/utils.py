import os
from PIL import Image
import cairo

def eps2pdf(eps_path, pdf_path):
    # Convert EPS to PDF using PIL
    with Image.open(eps_path) as img:
        img.save(pdf_path, 'PDF')

def pdf2svg(pdf_path, svg_path):
    """Convert PDF to SVG using pycairo."""
    # Open the output SVG file
    svg_surface = cairo.SVGSurface(svg_path, 595, 842)  # Define the size of your output file
    svg_context = cairo.Context(svg_surface)

    # Create a PDF surface for input
    pdf_surface = cairo.PDFSurface(pdf_path, 595, 842)  # Note: Normally, you'd use PDFSurface only for output

    # In a real scenario, you might render the PDF page to a temporary image surface and draw it on the SVG surface.
    # Here, we will mock this part as it's quite a bit more involved and Cairo alone doesn't offer direct PDF to SVG conversion.
    
    # Render PDF to SVG surface
    svg_context.set_source_surface(pdf_surface, 0, 0)
    svg_context.paint()
    svg_surface.finish()

# EPS to PDF
def convert_eps_to_pdf():
    directory = os.path.dirname(__file__)
    eps_filename = 'PathAndTrajectory.eps'
    pdf_filename = eps_filename.replace('.eps', '.pdf')
    eps_path = os.path.join(directory, eps_filename)
    pdf_path = os.path.join(directory, pdf_filename)
    eps2pdf(eps_path, pdf_path)
    print(f"Converted {eps_path} to {pdf_path}")

# PDF to SVG
def convert_pdf_to_svg():
    directory = os.path.dirname(__file__)
    pdf_filename = 'PathBeforeTrajectory.pdf'
    svg_filename = pdf_filename.replace('.pdf', '.svg')
    pdf_path = os.path.join(directory, pdf_filename)
    svg_path = os.path.join(directory, svg_filename)
    pdf2svg(pdf_path, svg_path)
    print(f"Converted {pdf_path} to {svg_path}")

if __name__ == "__main__":
    # convert_eps_to_pdf()
    convert_pdf_to_svg()

