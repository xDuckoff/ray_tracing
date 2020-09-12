## Порядок компиляции:
```
mkdir build    
cd build    
cmake −DCMAKE_BUILD_TYPE=Release ..     
make −j 4    
```

## Запуск:
`./rt −out <output_path> −scene <scene_number> −threads <threads>`
* output_path - путь к выходному изображению (относительный).
* scene_number - номер сцены от 1 до 2. 
  
Пример:
./rt -out 311_ivan_yellow.bmp -scene 2 -threads 1

## Сцены:


## Выполненные пункты:

Сцена 1:
* Базовая часть: +15 баллов
* Использование текстур: +1 балл
* Использование дополнительных геометрических приметивов: +2 балла (цилиндр, пирамида, плоскость, шар)
* Использования карты окружения: +1 балл (сферическая панорома звездного неба)
* Использование многопоточности: +2 балла 
* Преломление: +1 балл

* Субъективная реалистичность сцены: +2 балла
* Непридусмотренный бонус: +2 балла

Сцена 2:
* Использование поверхностей второго порядка: +4 балла (однополосный гиперболоид)
------------
Итого: 30 баллов
