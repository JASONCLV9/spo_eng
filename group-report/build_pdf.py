import subprocess, os

cwd = r'c:\Users\luisk\Desktop\E2 Examen Final\group-report'

def run(cmd):
    res = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    print('Command:', ' '.join(cmd), '-> Code:', res.returncode)
    if res.returncode != 0:
        print('STDERR:', res.stderr[-500:] if res.stderr else '(none)')
        print('STDOUT:', res.stdout[-800:] if res.stdout else '(none)')
    return res.returncode

run(['pdflatex', '-interaction=nonstopmode', 'group-report.tex'])
run(['bibtex', 'group-report'])
run(['pdflatex', '-interaction=nonstopmode', 'group-report.tex'])
run(['pdflatex', '-interaction=nonstopmode', 'group-report.tex'])

pdf_path = os.path.join(cwd, 'group-report.pdf')
if os.path.exists(pdf_path):
    print('group-report.pdf generated successfully! Size:', os.path.getsize(pdf_path), 'bytes')
else:
    print('Failed to generate PDF')
