# Исторические данные от 20 октября 2026

Эта папка хранит прежние отчёты, 239-костную / 7-клиповую модель в измерениях, исходные сравнения и промежуточные эксперименты. **Это не текущие результаты финального prepared asset.** Основной отчёт: [REPORT_RU.md](../../REPORT_RU.md).

`prepared_*.png`, прежние `gpu_*`, `assimp_prepared.json`, старые manifest/отчёты описывают предыдущую версию. `deformation_diagnostics.json`, `rigid_parts_diagnostics.json`, `cuff_weight_patch.json` и `prepared_fingers_glove_weight_test.png` включают промежуточные состояния; их нельзя использовать как доказательство весов окончательного файла. `glove_transfer_applied.json` — сохранённый канонический экземпляр удалённого точного дубля `glove_weight_patch.json`.

Прежние FBX/BLEND побайтово сохранены в [rollback ZIP](../../../../assets/models/backups/Raphtalia_prepared.pre_reaudit_2026-10-21.zip). Повторно измеренный baseline и GPU before находятся в [reaudit](../../reaudit/before.json). Исторические roundtrip/sweep здесь соответствуют этой модели; текущие результаты находятся выше, вне history.

Имена файлов и SHA256 при переносе проверены; полный список перемещений/удалений — [cleanup.json](../../reaudit/cleanup.json). Данные не удалялись только из-за возраста или похожего на временное имени. Отчёты в этой папке сохранены для прослеживания прежних утверждений, а не как альтернативная актуальная документация.
