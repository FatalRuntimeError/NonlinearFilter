using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;

namespace NonlinearFilter
{
	static class Plotter
	{
		public static string PlotterFolderPath = "C:/Users/Nikita/source/repos/NonlinearFilter/Plotter/";

		public static void DrawPlot(string csvData, int plotCount = 1, List<string> names = null)
		{
			string formattedNames = "";
			if (names != null)
			{
				foreach (string name in names)
					formattedNames = string.Concat(formattedNames, name + " ");
			}

			ProcessStartInfo start = new ProcessStartInfo();
			start.FileName = "C:/Users/Nikita/AppData/Local/Programs/Python/Python39/python.exe";
			start.Arguments = $"{PlotterFolderPath}DrawPlot.py {csvData} {plotCount} {formattedNames}";
			start.UseShellExecute = false;
			start.RedirectStandardOutput = true;
			using (Process process = Process.Start(start))
			{
				using (StreamReader reader = process.StandardOutput)
				{
					string result = reader.ReadToEnd();
					Console.Write(result);
				}
			}
		}

		public static void DrawFilter(string title, string csvData)
		{
			ProcessStartInfo start = new ProcessStartInfo();
			start.FileName = "C:/Users/Nikita/AppData/Local/Programs/Python/Python39/python.exe";
			start.Arguments = $"{PlotterFolderPath}DrawFilter.py {csvData} {title}";
			start.UseShellExecute = false;
			start.RedirectStandardOutput = true;
			using (Process process = Process.Start(start))
			{
				using (StreamReader reader = process.StandardOutput)
				{
					string result = reader.ReadToEnd();
					Console.Write(result);
				}
			}
		}
	}
}
