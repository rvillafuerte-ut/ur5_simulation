#obtener la data de /home/david/.ros/ur5_logs/
#se asume que la data esta en formato .csv y tiene cabeceras
#graficar con matplotlib x_des vs x_meas
import pandas as pd
import matplotlib.pyplot as plt
import argparse
import pathlib
import sys
import re
from datetime import datetime

_TS_REGEX = re.compile(r"(\d{8}_\d{6})")

def _extraer_timestamp(path: pathlib.Path):
	"""Devuelve datetime si encuentra patron YYYYMMDD_HHMMSS en el nombre, si no None."""
	m = _TS_REGEX.search(path.name)
	if not m:
		return None
	stamp = m.group(1)
	try:
		return datetime.strptime(stamp, "%Y%m%d_%H%M%S")
	except ValueError:
		return None

def listar_csv_logs(log_dir: pathlib.Path):
	if not log_dir.exists():
		print(f"Directorio no existe: {log_dir}")
		return []
	archivos = [p for p in log_dir.glob("*.csv") if p.is_file()]
	# Ordenar por timestamp extraído; si no tiene timestamp -> va al inicio
	archivos.sort(key=lambda p: (_extraer_timestamp(p) is None, _extraer_timestamp(p) or datetime.min))
	return archivos

def cargar_csv(path: pathlib.Path) -> pd.DataFrame:
	try:
		df = pd.read_csv(path)
		return df
	except Exception as e:
		print(f"Error leyendo {path}: {e}")
		return pd.DataFrame()

def graficar_posiciones(df: pd.DataFrame, guardar: bool = False, salida: pathlib.Path | None = None):
	columnas_requeridas = [
		"t",
		"x_des_x","x_des_y","x_des_z",
		"x_meas_x","x_meas_y","x_meas_z"
	]
	for c in columnas_requeridas:
		if c not in df.columns:
			print(f"Columna faltante en CSV: {c}")
			return

	t = df["t"].values
	x_des = df[["x_des_x","x_des_y","x_des_z"]].values
	x_meas = df[["x_meas_x","x_meas_y","x_meas_z"]].values

	fig, axes = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
	etiquetas = ["X", "Y", "Z"]
	for i, ax in enumerate(axes):
		ax.plot(t, x_des[:, i], label=f"x_des_{etiquetas[i].lower()}", linewidth=1.0)
		ax.plot(t, x_meas[:, i], label=f"x_meas_{etiquetas[i].lower()}", linewidth=1.0, alpha=0.8)
		ax.set_ylabel(f"Posición {etiquetas[i]} (m)")
		ax.grid(True, linestyle='--', alpha=0.4)
		ax.legend(loc="best", fontsize=8)
	axes[-1].set_xlabel("t (s)")
	fig.suptitle("Posición Deseada vs Medida")
	fig.tight_layout(rect=(0,0,1,0.97))

	if guardar:
		if salida is None:
			salida = pathlib.Path("posicion_vs_medida.png")
		fig.savefig(salida)
		print(f"Gráfico guardado en {salida}")
	else:
		plt.show()

def main():
	parser = argparse.ArgumentParser(description="Graficar x_des vs x_meas del último log CSV del UR5.")
	parser.add_argument("--dir", type=str, default=str(pathlib.Path.home()/".ros"/"ur5_logs"), help="Directorio de logs CSV")
	parser.add_argument("--archivo", type=str, default="", help="Ruta a CSV específico (si se proporciona se ignora --latest)")
	parser.add_argument("--latest", action="store_true", help="Usar el archivo más reciente automáticamente")
	parser.add_argument("--guardar", action="store_true", help="Guardar PNG en lugar de mostrar")
	parser.add_argument("--salida", type=str, default="", help="Ruta de salida para PNG")
	args = parser.parse_args()

	log_dir = pathlib.Path(args.dir)
	if args.archivo:
		csv_path = pathlib.Path(args.archivo)
		if not csv_path.exists():
			print(f"Archivo especificado no existe: {csv_path}")
			sys.exit(1)	
	else:
		archivos = listar_csv_logs(log_dir)
		print(archivos)
		if not archivos:
			print(f"No se encontraron CSV en {log_dir}")
			sys.exit(1)
		if args.latest or not args.archivo:
			csv_path = archivos[-1]  # último según orden temporal real
		else:
			csv_path = archivos[-1]

	print(f"Usando CSV: {csv_path}")
	df = cargar_csv(csv_path)
	if df.empty:
		print("DataFrame vacío, abortando")
		sys.exit(1)

	salida = pathlib.Path(args.salida) if args.salida else None
	graficar_posiciones(df, guardar=args.guardar, salida=salida)

if __name__ == "__main__":
	main()


