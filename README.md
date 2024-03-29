# SpotMicro-Leika
<div id="top"></div>
<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/runeharlyk/SpotMicro-Leika">
    <img src="assets/logo.jpg" alt="Logo" width="400" height="400">
  </a>

<h3 align="center">Spot Micro - Leika</h3>

  <p align="center">
    This project is a small quadruped robot, inspired by boston dynamic spot mini. 
    <br />
    <a href="https://github.com/runeharlyk/SpotMicro-Leika"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <!--<a href="https://github.com/runeharlyk/SpotMicro-Leika">View Demo</a>
    ·
    <a href="https://github.com/runeharlyk/SpotMicro-Leika/issues">Report Bug</a>
    ·
    <!--<a href="https://github.com/runeharlyk/SpotMicro-Leika/issues">Request Feature</a>-->
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project
This project is my attempt on building and assembling [SpotMicro](https://github.com/michaelkubina/SpotMicroESP32). I chose a Raspberry pi as the brain of robot, and therefore this repos mainly focused on the code part of the build as some amazing people have designed the model very well.

<p align="right">(<a href="#top">back to top</a>)</p>


## Built With
* React.js
* threading
* tornado
* gpiozero

<p align="right">(<a href="#top">back to top</a>)</p>

### 3D printed body
* [robjk reinforced shoulder remix](https://www.thingiverse.com/thing:4937631)
* [Kooba SpotMicroESP32 remix](https://www.thingiverse.com/thing:4559827)
* [KDY0532 original design](https://www.thingiverse.com/thing:3445283)


### Electronics

* Raspberry pi 3b
* PCA9685 servo board
* 12x 20kg(or higher) servo motors
* MPU6050 
* 20A DC-DC Buck Converter
* 2x HC-SR04 Ultrasonic Distance Sensor
* 0.96" SD1306 OLED diplay 
* ACS712 current sensor
* ADS1115 12 bit analog to digital converter
* DHT22 temperature and humidity sensor

<p align="right">(<a href="#top">back to top</a>)</p>

# Getting started

### Prerequisites

This is an example of how to list things you need to use the software and how to install them.
1. Install headless raspbian on your raspberry pi

### Installation

1. Clone the repo
   ```sh
   git clone https://github.com/runeharlyk/SpotMicro-Leika.git
   ```
2. Move to working directory
   ```sh
   cd SpotMicro-Leika
   ```
3. Install dependencies
   ```sh
   pip install -r requirements.txt
   ```
4. Run the main program
   ```sh
   python spot.py
   ```

<p align="right">(<a href="#top">back to top</a>)</p>


<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Rune Harlyk - [@runeharlyk](https://twitter.com/runeharlyk)

Project Link: [https://github.com/runeharlyk/SpotMicro-Leika](https://github.com/runeharlyk/SpotMicro-Leika)

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/github_username/repo_name.svg?style=for-the-badge
[contributors-url]: https://github.com/github_username/repo_name/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/github_username/repo_name.svg?style=for-the-badge
[forks-url]: https://github.com/github_username/repo_name/network/members
[stars-shield]: https://img.shields.io/github/stars/github_username/repo_name.svg?style=for-the-badge
[stars-url]: https://github.com/github_username/repo_name/stargazers
[issues-shield]: https://img.shields.io/github/issues/github_username/repo_name.svg?style=for-the-badge
[issues-url]: https://github.com/github_username/repo_name/issues
[license-shield]: https://img.shields.io/github/license/github_username/repo_name.svg?style=for-the-badge
[license-url]: https://github.com/github_username/repo_name/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/linkedin_username
[product-screenshot]: images/screenshot.png
[Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com 
