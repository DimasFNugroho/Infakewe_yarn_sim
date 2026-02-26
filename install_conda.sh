set -euo pipefail

mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -p ~/miniconda3
rm -f ~/miniconda3/miniconda.sh
echo "Miniconda installed at ~/miniconda3"
echo "Next: run 'source ~/miniconda3/bin/activate' or add ~/miniconda3/bin to your PATH."
