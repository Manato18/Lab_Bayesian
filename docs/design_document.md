# Bayesian Simulation Refactoring Design Document

## 1. 概要 (Overview)

このプロジェクトはコウモリのエコーロケーションに基づく空間認知モデルをシミュレーションするコードです。
現在は一つの大きなファイル（bayse_sample.py）で実装されていますが、機能別にモジュールを分割し、
責務を明確化することでコードの可読性、保守性、拡張性を向上させることが目的です。

将来的には、「事後分布に基づく動的回避経路」の機能を実装する予定です。

## 2. システム構成 (System Architecture)
