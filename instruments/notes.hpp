namespace PICO {
  class Notes {
    private: static
    const int octave_arr_index = 12;

    const float zeroth_octave[octave_arr_index] = {
      16.35,
      17.32,
      18.35,
      19.45,
      20.6,
      21.83,
      23.12,
      24.5,
      25.96,
      27.5,
      29.14,
      30.87
    };
    const float first_octave[octave_arr_index] = {
      32.7,
      34.65,
      36.71,
      38.89,
      41.2,
      43.65,
      46.25,
      49,
      51.91,
      55,
      58.27,
      61.74
    };
    const float second_octave[octave_arr_index] = {
      65.41,
      69.3,
      73.42,
      77.78,
      82.41,
      87.31,
      92.5,
      98,
      103.8,
      110,
      116.5,
      123.5
    };
    const float third_octave[octave_arr_index] = {
      130.8,
      138.6,
      146.8,
      155.6,
      164.8,
      174.6,
      185,
      196,
      207.7,
      220,
      233.1,
      246.9
    };
    const float fourth_octave[octave_arr_index] = {
      261.6,
      277.2,
      293.7,
      311.1,
      329.6,
      349.2,
      370,
      392,
      415.3,
      440,
      466.2,
      493.9
    };
    const float fifth_octave[octave_arr_index] = {
      523.3,
      554.4,
      587.3,
      622.3,
      659.3,
      698.5,
      740,
      784,
      830.6,
      880,
      932.3,
      987.8
    };
    const float sixth_octave[octave_arr_index] = {
      1047,
      1109,
      1175,
      1245,
      1319,
      1397,
      1480,
      1568,
      1661,
      1760,
      1865,
      1976
    };
    const float seventh_octave[octave_arr_index] = {
      2093,
      2217,
      2349,
      2489,
      2637,
      2794,
      2960,
      3136,
      3322,
      3520,
      3729,
      3951
    };
    const float eighth_octave[octave_arr_index] = {
      4186,
      4435,
      4699,
      4978,
      5274,
      5588,
      5920,
      6272,
      6645,
      7040,
      7459,
      7902
    };

    const float get_frequency(const int Note,
      const int Octave) {
      if (Octave > 8 || note > 11)
        return 0;

      switch (Octave) {
      case 0:
        return zeroth_octave[Note];
        break;
      case 1:
        return first_octave[Note];
        break;
      case 2:
        return second_octave[Note];
        break;
      case 3:
        return third_octave[Note];
        break;
      case 4:
        return fourth_octave[Note];
        break;
      case 5:
        return fifth_octave[Note];
        break;
      case 6:
        return sixth_octave[Note];
        break;
      case 7:
        return seventh_octave[Note];
        break;
      case 8:
        return eighth_octave[Note];
        break;
      default:
        return 0;
        break;
      }
      return 0;
    }

    public: enum List {
      C = 0,
        CS = 1,
        D = 2,
        DS = 3,
        E = 4,
        F = 5,
        FS = 6,
        G = 7,
        GS = 8,
        A = 9,
        AS = 10,
        B = 11
    };

    // returned freqyency from all the functions here that the user can access
    float frequency = 0;
    int note = 0; // note choosen by th euser
    int octave = 0; // octave choosen by the user

    Notes(const int note,
      const int octave) {
      this -> note = note;
      this -> octave = octave;
      frequency = get_frequency(this -> note, this -> octave);
    }

    const char * toString() {
      switch (note) {
      case List::C:
        return "C";
        break;
      case List::CS:
        return "C#";
        break;
      case List::D:
        return "D";
        break;
      case List::DS:
        return "D#";
        break;
      case List::E:
        return "E";
        break;
      case List::F:
        return "F";
        break;
      case List::FS:
        return "F#";
        break;
      case List::G:
        return "G";
        break;
      case List::GS:
        return "G#";
        break;
      case List::A:
        return "A";
        break;
      case List::AS:
        return "A#";
        break;
      case List::B:
        return "B";
        break;
      default:
        return "";
        break;
      }
      return "";
    }
  };
  typedef Notes Note;
}
