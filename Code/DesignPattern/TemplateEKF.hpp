#ifndef __BASE_BF_H__
#define __BASE_BF_H__

#include "bfl/filter/extendedkalmanfilter.h"
#include "bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h"
#include "bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h"
#include "bfl/pdf/analyticconditionalgaussian.h"
#include "bfl/pdf/linearanalyticconditionalgaussian.h"
#include "Eigen/Dense"

#include <unordered_map>
#include <memory>


namespace keenon_ekf
{
	using namespace std;
	using namespace BFL;
	using namespace MatrixWrapper;

	template <typename SysPdf = BFL::AnalyticConditionalGaussianAdditiveNoise, 
		typename SysModel = BFL::AnalyticSystemModelGaussianUncertainty, 
		typename MeasPdf = BFL::LinearAnalyticConditionalGaussian, 
		typename MeasModel = BFL::LinearAnalyticMeasurementModelGaussianUncertainty, 
		typename PriorPdf = BFL::Gaussian, 
		typename Filter = BFL::ExtendedKalmanFilter>
	class BaseBF
	{
	protected:
		using MeasModelType = std::string;

	public:
		virtual ~BaseBF() = default;

		void setSystemModel(const ColumnVector& sysNoise_Mu, const SymmetricMatrix& sysNoise_Cov)
		{
			Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
			sys_pdf_ = std::make_shared<SysPdf>(system_Uncertainty);
			sys_model_ = std::make_shared<SysModel>(sys_pdf_.get());
		}

		void setMeasureModel(const MeasModelType& type, const Matrix& Himu, const ColumnVector& measNoise_Mu, const SymmetricMatrix& measNoise_Cov)
		{
			Gaussian measurement_Uncertainty(measNoise_Mu, measNoise_Cov);
			meas_pdf_list_[type] = std::make_shared<MeasPdf>(Himu, measurement_Uncertainty);
			meas_model_list_[type] = std::make_shared<MeasModel>(meas_pdf_list_[type].get());
		}

		void setPriorForFilter(const ColumnVector& prior_Mu, const SymmetricMatrix& prior_Cov)
		{
			prior_ = std::make_shared<PriorPdf>(prior_Mu, prior_Cov);
			filter_ = std::make_shared<Filter>(prior_.get());
		}

		void SystemUpdate(const ColumnVector& control)
		{
			filter_->Update(sys_model_.get(), control);
		}

		void MeasureUpdate(const MeasModelType& type, const ColumnVector& meas)
		{
			filter_->Update(meas_model_list_[type].get(), meas);
		}

		void setAdditiveNoiseCovarianceForMeasure(const MeasModelType& type, const SymmetricMatrix& covar)
		{
			meas_pdf_list_[type]->AdditiveNoiseSigmaSet(covar);
		}

		ColumnVector getExpectedValue() const
		{
			return filter_->PostGet()->ExpectedValueGet();
		}

		SymmetricMatrix getCovariance() const
		{
			return filter_->PostGet()->CovarianceGet();
		}

		std::shared_ptr<SysPdf> getSystemPdf() const
		{
			return sys_pdf_;
		}

		std::shared_ptr<MeasPdf> getMeasurePdf(const MeasModelType& type)
		{
			return meas_pdf_list_[type];
		}

	protected:
		std::shared_ptr<SysPdf> sys_pdf_;
		std::shared_ptr<SysModel> sys_model_;

		std::unordered_map<MeasModelType, std::shared_ptr<MeasPdf>> meas_pdf_list_;
		std::unordered_map<MeasModelType, std::shared_ptr<MeasModel>> meas_model_list_;

		std::shared_ptr<PriorPdf> prior_;
		std::shared_ptr<Filter> filter_;
	};

	using OdomEKF = BaseBF<BFL::AnalyticConditionalGaussianAdditiveNoise>;
	using LabelEKF = BaseBF<BFL::AnalyticConditionalGaussianAdditiveNoise>;
}

#endif
