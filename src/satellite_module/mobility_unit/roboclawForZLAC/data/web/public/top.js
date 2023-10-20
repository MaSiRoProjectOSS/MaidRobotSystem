if (!JS_Ctrl) {
    var JS_Ctrl = {
        m_list: ["_it", "_ic", "_iv", "_tr", "_sc", "_sr", "_pr", "_pg", "_ps", "_pf", "_pfd"],
        e_list: ["_ess", "_esu", "_ec", "_eov", "_euv", "_eh", "_eol", "_ee", "_enc"],
        c_list: ["c_ma", "c_sa", "c_sd", "c_pl", "c_sm", "c_tm"],
        f_list: ["_c_gkp", "_c_gki", "_c_gkd", "_c_gkf"],
        m_name: ["NOT_INITIALIZED", "POSITION[PULSE]", "POSITION[DIGITAL]", "POSITION[ANALOG]", "SPEED[DIGITAL]", "SPEED[ANALOG]", "TORQUE[DIGITAL]", "TORQUE[ANALOG]"],
        s_name: ["BEGIN", "ERROR", "CLEAR_RECEIVE", "disconnected", "CONNECTED", "MODE", "EMERGENCY", "CRC-ERROR"],
        s_m: ["", "(LEFT)", "(RIGHT)"],
        s_o: ["", "[detected] ", "[RESOLVED] "],
        s_e: ["", "", "", "", "",
            " (over current)", " (over voltage)", " (under voltage)",
            " (encoder error)", " (overheat)", " (overload)", " (disconnected)"],
        get_data: null,
        timerId: null,
        l_max: 25,
        f_log_d: false,
        loading: false,
        timerInterval: 500,
        save_config: function () {
            JS_AJAX.get('/set/save').then(
                ok => alert('SAVE config \n<Successful>')
                , error => alert('SAVE config \n<Failed>')
            );
        },
        s_num: function (elm, val) {
            let el = document.getElementById(elm);
            if (null != el) {
                if (el.value != val) {
                    el.value = val;
                }
            }
        },
        set_gain: function (elm, key, target) {
            let add_param = key + "=" + elm.value;

            JS_AJAX.get('/set/setting?' + ((0 == target) ? "le=1" : "re=1") + "&" + add_param);
        },
        set_inverted: function (elm, key, target) {
            let add_param = key + "=" + elm.value;
            JS_AJAX.get('/set/setting?' + ((0 == target) ? "le=1" : "re=1") + "&" + add_param);
        },
        log_filter: function () {
            var sw = document.getElementById("swFilterDirect");
            JS_Ctrl.f_log_d = !JS_Ctrl.f_log_d;
            sw.checked = JS_Ctrl.f_log_d;
        },
        log_ch: function () {
            var sw = document.getElementById("swLogSize");
            sw.checked = !sw.checked;
            JS_Ctrl.l_max = (true == sw.checked) ? 1000 : 25;
            document.getElementById("sfd_cnt").innerHTML = "Log Size : " + JS_Ctrl.l_max;
        },
        s_yn: function (elm, val) {
            let el = document.getElementById(elm);
            if (null != el) {
                val = (val == 1) ? "Y" : "N";
                if (el.innerHTML != val) {
                    el.innerHTML = val;
                }
            }
        },
        stop: function (elm, val) {
            var sw = document.getElementById(elm);
            let ch = sw.checked ? "?off=1" : "";
            JS_AJAX.get('/set/stop' + ch);
        },
        s_stop: function (val) {
            var sw = document.getElementById('swEmergency');
            if (sw.checked != val) {
                sw.checked = val;
            }
        },
        s_pa: function (elm, val) {
            let el = document.getElementById(elm);
            if (null != el) {
                val = (val == 1) ? "absolute" : "relative";
                if (el.innerHTML != val) {
                    el.innerHTML = val;
                }
            }
        },
        s_elm: function (elm, val) {
            let el = document.getElementById(elm);
            if (null != el) {
                if (el.innerHTML != val) {
                    el.innerHTML = val;
                }
            }
        },
        s_fl: function (elm, val) {
            let el = document.getElementById(elm);
            if (null != el) {
                val = (val == 1) ? "Y" : "N";
                if (el.innerHTML != val) {
                    el.innerHTML = val;
                    if ("Y" == val) {
                        el.classList.add("td_warning");
                    } else {
                        el.classList.remove("td_warning");
                    }
                }
            }
        },
        z_p: function (NUM, LEN) {
            return (Array(LEN).join('0') + NUM).slice(-LEN);
        },
        s_t: function (val) {
            if (0 != val) {
                return (Number(val) / 1000).toFixed(3);
            } else { return "--"; }
        },
        ins_ul: function (func, elm, val) {
            let ul = document.getElementById(elm);
            let new_lest = 0;
            if (null != ul.firstChild) {
                new_lest = ul.firstChild.value;
            }
            for (i = val.length - 1; 0 <= i; i--) {
                buf = Number(((val[i][0] * 100000) + val[i][1]).toFixed(0));
                if (new_lest >= buf) { continue; }
                let li = document.createElement("li");
                li.value = buf;
                li.innerHTML = func(val, i);

                if ("" != li.innerHTML) {
                    ul.prepend(li);
                }
            }
            if (null != ul.children) {
                while (JS_Ctrl.l_max < ul.children.length) {
                    ul.lastChild.remove();
                }
            }
        },
        s_log: function (val, i) {
            return val[i][0].toFixed(3) + "s : "
                + JS_Ctrl.z_p(val[i][1].toString(16), 2) + ","
                + JS_Ctrl.z_p(val[i][2].toString(16), 2) + ","
                + JS_Ctrl.z_p(val[i][3].toString(16), 2);
        },
        s_slog: function (val, i) {
            return val[i][0].toFixed(3) + "s : "
                + JS_Ctrl.s_o[val[i][3]] + JS_Ctrl.s_name[val[i][1]] + " " + JS_Ctrl.s_m[val[i][2]]
                + ((5 == val[i][1]) ? ("/ " + JS_Ctrl.m_name[val[i][4]]) : "")
                + ((1 == val[i][5]) ? JS_Ctrl.s_e[5] : "")
                + ((1 == val[i][6]) ? JS_Ctrl.s_e[6] : "")
                + ((1 == val[i][7]) ? JS_Ctrl.s_e[7] : "")
                + ((1 == val[i][8]) ? JS_Ctrl.s_e[8] : "")
                + ((1 == val[i][9]) ? JS_Ctrl.s_e[9] : "")
                + ((1 == val[i][10]) ? JS_Ctrl.s_e[10] : "")
                + ((1 == val[i][11]) ? JS_Ctrl.s_e[11] : "");
        },
        s_d_log: function (val, i) {
            if (0x25 != val[i][2] && (true == JS_Ctrl.f_log_d)) {
                return "";
            } else {
                return val[i][0].toFixed(3) + "s : "
                    + "0x" + JS_Ctrl.z_p(val[i][1].toString(16), 2)
                    + "[" + "0x" + JS_Ctrl.z_p(val[i][2].toString(16), 2) + "/size(" + JS_Ctrl.z_p(val[i][3], 2) + ")] : "
                    + JS_Ctrl.z_p(val[i][4].toString(16), 2) + ","
                    + JS_Ctrl.z_p(val[i][5].toString(16), 2) + ","
                    + JS_Ctrl.z_p(val[i][6].toString(16), 2) + ","
                    + JS_Ctrl.z_p(val[i][7].toString(16), 2) + ","
                    + JS_Ctrl.z_p(val[i][8].toString(16), 2) + ","
                    + JS_Ctrl.z_p(val[i][9].toString(16), 2) + ","
                    + JS_Ctrl.z_p(val[i][10].toString(16), 2) + ","
                    + JS_Ctrl.z_p(val[i][11].toString(16), 2) + ","
                    + JS_Ctrl.z_p(val[i][11].toString(16), 2) + ","
                    + JS_Ctrl.z_p(val[i][12].toString(16), 2)
                    ;
            }
        },
        make: function (v) {
            if (null != v) {
                if ("OK" == v.result) {
                    if (null != JS_Ctrl.get_data) {
                        if (JS_Ctrl.get_data.status.KEY != v.status.KEY) {
                            JS_Ctrl.init();
                        }
                    }
                    JS_Ctrl.get_data = v;
                    for (i = 0; i < JS_Ctrl.m_list.length; i++) {
                        JS_Ctrl.s_elm("l" + JS_Ctrl.m_list[i], v.status.data.left[0][i]);
                        JS_Ctrl.s_elm("r" + JS_Ctrl.m_list[i], v.status.data.right[0][i]);
                    }
                    for (i = 0; i < JS_Ctrl.e_list.length; i++) {
                        JS_Ctrl.s_fl("l" + JS_Ctrl.e_list[i], v.status.data.left[1][i]);
                        JS_Ctrl.s_fl("r" + JS_Ctrl.e_list[i], v.status.data.right[1][i]);
                    }
                    for (i = 0; i < JS_Ctrl.f_list.length; i++) {
                        JS_Ctrl.s_num("l" + JS_Ctrl.f_list[i], v.status.data.left[2][i]);
                        JS_Ctrl.s_num("r" + JS_Ctrl.f_list[i], v.status.data.right[2][i]);
                    }
                    JS_Ctrl.s_elm("c_mo", JS_Ctrl.m_name[v.status.data.both[0][0]]);
                    JS_Ctrl.s_pa("c_pa", v.status.data.both[0][1]);
                    JS_Ctrl.s_yn("c_fr", v.status.data.both[0][2]);
                    JS_Ctrl.s_stop((1 == v.status.data.both[0][3]) ? true : false);
                    JS_Ctrl.s_yn("l_c_iLe", v.status.data.both[0][4]);
                    JS_Ctrl.s_yn("r_c_iLe", v.status.data.both[0][5]);
                    for (i = 0; i < JS_Ctrl.c_list.length; i++) {
                        JS_Ctrl.s_elm(JS_Ctrl.c_list[i], v.status.data.both[1][i]);
                    }
                    JS_Ctrl.ins_ul(JS_Ctrl.s_d_log, "log_ul_d", v.status.data.log[0]);
                    JS_Ctrl.ins_ul(JS_Ctrl.s_log, "log_ul_l", v.status.data.log[1]);
                    JS_Ctrl.ins_ul(JS_Ctrl.s_log, "log_ul_r", v.status.data.log[2]);
                    JS_Ctrl.ins_ul(JS_Ctrl.s_slog, "log_ul_s", v.status.data.log[3]);
                }
            }
            JS_Ctrl.loading = false;
        },
        clear_log: function (elm) {
            let ul = document.getElementById(elm);
            if (null != ul.children) {
                while (0 < ul.children.length) {
                    ul.lastChild.remove();
                }
            }
        },
        init: function () {
            JS_Ctrl.s_stop(false);
            JS_Ctrl.clear_log("log_ul_d");
            JS_Ctrl.clear_log("log_ul_l");
            JS_Ctrl.clear_log("log_ul_r");
            JS_Ctrl.clear_log("log_ul_s");
        },
        err: function (v) {
            console.error("JS_Ctrl : " + v.status.messages);
            JS_Ctrl.loading = false;
        },
        interval: function () {
            if (false == JS_Ctrl.loading) {
                JS_Ctrl.loading = true;
                JS_AJAX.get('/get/motor').then(
                    ok => JS_Ctrl.make(ok)
                    , error => JS_Ctrl.err(error)
                );
            }
        }
    }
}

window.onload = function () {
    JS_Ctrl.timerId =
        setInterval(JS_Ctrl.interval, JS_Ctrl.timerInterval);
};
window.onunload = function () {
    if (null != JS_Ctrl.timerId) {
        clearInterval(JS_Ctrl.timerId);
    }
};
