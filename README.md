<p align="center"><img src="doc/img/PRO.png" width="70%" /><br><br></p>

-----------------
[![GitHub CI](../../actions/workflows/buildsCI.yaml/badge.svg?branch=development)](../../actions/workflows/buildsCI.yaml)

## 概要
**Stereo PRO SDK 3.2.0** はSiNGRAY Stereo PROカメラ用クロスプラットフォームライブラリです。

このSDKで、デプスデータとカラー画像のストリーミングの取得が可能で、内部パラメータと外部パラメータを提供します。
ライブラリは合成ストリーム（点群データ、デプス画像をカラー画像と位置合を合わせるデータ）と、ストリーミングセッションの記録と再生のための組込サポートも提供しています。

このライブラリの使用に必要なハードウェアと開発キットは、下記のサイトからお問合せを頂ければご購入いただけます。 [https://singray-stereo.com/](https://singray-stereo.com/).
Stereo PRO　開発用のSDKなどは下記のサイトにてご確認いただけます。
[https://singray-stereo.com/category/support/prosoftware/](https://singray-stereo.com/category/support/prosoftware/)


## ダウンロードとインストール 
* **ダウロード** - Stereo PRO SDK、Viewer、Depth デプスのテストツールなどの最新版は、次のサイトからダウンロードできます: [**latest releases**](https://singray-stereo.com/category/support/prosoftware/). 対応できるプラットフォーム、新機能、既知の問題、ファームウェアのアップグレード方法などは [**release notes**](https://singray-stereo.com/category/support/prodocuments/) を確認してください。

* **インストール** - SDKをインストールして、Stereo PROカメラをPCに接続した後、アプリケーションの作成を開始することができます。

> **サポート & 問題解決**: 製品サポート(ex.デバイスに関する質問やデバイスに問題がある)が必要な場合、[FAQ]を参照してください。(https://singray-stereo.com/stereo-pro-faq/)
> FAQで問題が解決されない場合は公式フォーラム [フォーラム](https://singray-stereo.com/forums/forum/stereo-pro-forum/) と [ドキュメント資料](https://singray-stereo.com/category/support/prodocuments/) をチェックしてください。
上記の方法で質問の答えが見つからない場合、 [問合せフォーム](https://singray-stereo.com/inform/)の記入またはお問合せ [メール](info@singray-stereo.com)を送ってください。

## SDKの内容:
| 項目 | 説明 | ダウンロードリンク|
| ------- | ------- | ------- |
| **Stereo PRO Viewer** | このアプリケーションで、Stereo PROにアクセスすると、デプスストリームの表示、SLAM/CSLAM、IMU、RGB、RGB-D、カメラ設定の変更、Advanced contorol、デプスの可視化とポスト処理の有効化など、さまざまな機能があります。 | [**Stereo PRO viewer.exe**](https://singray-stereo.com/category/support/prosoftware/) |
| **[Code Samples](./examples)** |これらのサンプルは、SDKを使用してカメラにアクセスするコードスニペットをアプリケーションに簡単に入れる方法を紹介しています。 [**C++ examples**](./examples) | [**Stereo PRO SDK.exe**](https://singray-stereo.com/category/support/prosoftware/)に含まれています。 |
| **Wrappers** | Python,C++,Andorid API


## 始めましょう!

当社のライブラリはStereo PROカメラ専用の高レベルのAPIを提供しています。
下記のコードクリップは、フレーム転送を開始し、ピクセルのデプス値を抽出する方法を示しています。:

```cpp
//デバイスを取得する
	auto devices = xv::getDevices(10., json);

//ToFカメラストリームを設定して起動する
	auto device = devices.begin()->second;
    device->tofCamera()->start();

//ToFストリームコールバック関数を登録する
	device->tofCamera()->registerCallback([&](xv::DepthImage const & tof) 
	{
	    //デプスデータ配列を取得する
        const auto tmp_d = reinterpret_cast<int16_t const*>(tof->data.get());
        //ピクセル中心からターゲット点までの距離を取得する
        float depth=tmp_d[tof->height / 2 * tof->width + tof->width / 2];
		    
	}
このライブラリの詳細については [examples](./examples)に従い、 [documentation](./doc) で確認してください。

## Contributing
In order to contribute to Stereo PRO SDK, please follow our [contribution guidelines](CONTRIBUTING.md).
