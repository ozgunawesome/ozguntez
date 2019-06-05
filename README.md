## **YTU Electronics & Comm Eng Undergrad Thesis**
## **Özgün Ayaz, July 2011**

I wrote this Verilog IP to accelerate [chirp estimation](https://en.wikipedia.org/wiki/Chirp) using the [Fractional Fourier Transform algorithm](https://en.wikipedia.org/wiki/Fractional_Fourier_transform) on an Altera FPGA device. 

### **Preface**

Chirp estimation is commonly used on both stationary and airborne radar equipment to determine both the speed and trajectory of moving objects on a 3D coordinate space. If you're interested in the math involved, see more here: https://e-reports-ext.llnl.gov/pdf/830491.pdf

The research behind this project has also been published in an IEEE journal: https://ieeexplore.ieee.org/document/5929841 

### **About the repo**

This repo is mostly for historical and reference purposes since it hasn't been touched in 10 years. I ran it on an [Altera DE2 board](https://www.terasic.com.tw/cgi-bin/page/archive.pl?No=30) (which is discontinued so they don't even sell it anymore) but it should run on any FPGA with enough gates.

Feel free to browse the repo -- especially if you're an FPGA beginner. There are some parts I did really well and there are others that I'd done.. not so well. As said, for reference only.

### **Limited liability**

This repo is published as-is, which means the author takes no responsibility whatsoever. Building this into a bitstream file and loading it on an FPGA could result in anything ranging from your house catching fire, to random military aircraft acquiring missile lock on you. Be wary.

#### THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

### **Licensing**

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)


This repository is licensed under **Creative Commons CC BY-NC-SA 4.0** 

**which means you are free to:**

* **Share** — copy and redistribute the material in any medium or format
* **Adapt** — remix, transform, and build upon the material

**under the following terms:**

* **Attribution** — You must give **appropriate credit**, provide a link to the license, and **indicate if changes were made**. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
* **NonCommercial** — You may not use the material for commercial purposes.
* **ShareAlike** — If you remix, transform, or build upon the material, you must distribute your contributions under the same license as the original.
* **No additional restrictions** — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.

For more information kindly refer to `LICENSE.md` 

