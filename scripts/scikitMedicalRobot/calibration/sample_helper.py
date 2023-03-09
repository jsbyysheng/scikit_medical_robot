from IPython.display import display, Markdown, clear_output
import ipywidgets as widgets

class jupyter_quick_sample:
    def __init__(self, name, sample_handler, eval_handler, clear_samples_handler, save_samples_handler) -> None:
        self.name = name
        self.sample_handler = sample_handler
        self.eval_handler = eval_handler
        self.clear_samples_handler = clear_samples_handler
        self.save_samples_handler = save_samples_handler

        self.button_sample = widgets.Button(description='Sample')
        self.button_resample_last = widgets.Button(description='Resample Last')
        self.button_delete_last = widgets.Button(description='Delete Last')
        self.button_clear = widgets.Button(description='Clear All')
        self.button_save = widgets.Button(description='Save')
        self.out = widgets.Output()

        self.datas = []

    def on_button_sample_clicked(self, e):
        with self.out:
            clear_output()
            ret = self.sample_handler(len(self.datas))
            if ret is not None:
                self.datas.append(ret)
            print(f"N Sample: {len(self.datas)}")
            self.eval_handler(self.datas)

    def on_button_resample_last_clicked(self, e):
        with self.out:
            clear_output()
            ret = self.sample_handler(len(self.datas))
            if ret is not None:
                if len(self.datas) >= 1:
                    del self.datas[-1]
                self.datas.append(ret)
            print(f"N Sample: {len(self.datas)}")
            self.eval_handler(self.datas)

    def on_button_delete_last_clicked(self, e):
        with self.out:
            clear_output()
            if len(self.datas) >= 1:
                del self.datas[-1]
            print(f"N Sample: {len(self.datas)}")
            self.eval_handler(self.datas)

    def on_button_clear_clicked(self, e):
        with self.out:
            clear_output()
            self.clear_samples_handler(self.datas)
            self.datas = []
            print(f"N Sample: {len(self.datas)}")
        
    def on_button_save_clicked(self, e):
        with self.out:
            clear_output()
            self.save_samples_handler(self.datas, self.name)

    def run_sample(self):
        self.button_sample.on_click(self.on_button_sample_clicked)
        self.button_resample_last.on_click(self.on_button_resample_last_clicked)
        self.button_delete_last.on_click(self.on_button_delete_last_clicked)
        self.button_clear.on_click(self.on_button_clear_clicked)
        self.button_save.on_click(self.on_button_save_clicked)

        return widgets.VBox([widgets.HBox([self.button_sample, self.button_resample_last, self.button_delete_last, self.button_clear, self.button_save]), self.out])