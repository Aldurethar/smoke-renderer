#include <vector>
#include <fstream>

template<typename F, size_t AXES>


bool writeField(const std::string& filename, const F* field, const size_t (&dims)[AXES])
{
	std::ofstream f(filename.c_str(), std::ofstream::out | std::ofstream::binary);
	if(!f) return false;

	unsigned long long nDims = AXES;
	f.write((char*)&nDims, sizeof(nDims));
	if(!f) return false;

	unsigned long long ullDims[AXES];
	for(size_t ax = AXES; ax-- != 0;)
		ullDims[ax] = dims[ax];
	f.write((char*)ullDims, sizeof(ullDims));
	if(!f) return false;

	f.write((const char*)field, sizeof(F)*prod(dims));
	if(!f) return false;

	return true;
}

bool readField(
	const std::string& filename,
	std::vector<float>& field,
	std::vector<size_t>& dims
)
{
	// assuming little-endian architecture
	
	std::ifstream f(filename.c_str(), std::ifstream::in | std::ifstream::binary);
	if (!f) {
		qDebug("Could not construct filestream!");
		return false;
	}
	
	// read number of dimensions
	unsigned long long nDims;
	f.read((char*)&nDims, sizeof(nDims));
	if (!f || (!dims.empty() && dims.size() != nDims)) {
		qDebug("Could not read number of Dimensions!");
		return false;
	}
	
	// read dimensions of field
	std::vector<unsigned long long> dims_; dims_.resize((size_t)nDims);
	f.read((char*)&dims_[0], sizeof(unsigned long long) * (size_t)nDims);
	if(!f) return false;
	
	if(dims.empty())
	{
		dims.resize((size_t)nDims);
		for(size_t i = 0; i < (size_t)nDims; ++i)
			dims[i] = (size_t)dims_[i];
	}
	else
	{
		for(size_t i = 0; i < (size_t)nDims; ++i)
			if(dims[i] != dims_[i])
				return false;
	}

	size_t newSize = 1;
	for (int i = 0; i < dims.size(); i++) {
		newSize *= dims[i];
	}

	field.resize(newSize);
	f.read((char*)&field[0], newSize * sizeof(float));
	if(!f) return false;
	
	return true;
}
