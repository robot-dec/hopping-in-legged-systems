import matplotlib.pyplot as plt


def plot_histogram(t, x, y, title, x_labels={}, y_labels={}):
    for i, data in enumerate(x):
        if i in x_labels.keys():
            plt.plot(t, data, label=x_labels[i])

    for i, data in enumerate(y):
        if i in y_labels.keys():
            plt.plot(t, data, label=y_labels[i])

    plt.legend()
    plt.title(title)

    plt.savefig("media/" + title.lower().replace(" ", "_") + ".png")

    plt.show()


def plot_hopping(t, x, y, title):
    x_labels = {
        3: "foot altitude",
        4: "leg length"
    }
    y_labels = {
        3: "hip altitude",
    }

    plot_histogram(t, x, y, title, x_labels, y_labels)
