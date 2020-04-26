using System;

namespace raytracing
{
    /// <summary>
    /// Klasa predstvlja scenu kod modela crtanja slike pomocu ray tracinga. Sastoji
    /// se od izvora svjetlosti i konacnog broja objekata.
    /// </summary>
    public class Scene
    {
        const int MAXDEPTH = 5; //maksimalna dubina rekurzije
        private int numberOfObjects;
        private Sphere[] sphere;
        private Point lightPosition;
        private ColorVector backgroundColors = new ColorVector(0, 0, 0);
        private ColorVector light = new ColorVector((float)1, (float)1, (float)1);
        private ColorVector ambientLight = new ColorVector((float)0.5, (float)0.5, (float)0.5);

        /// <summary>
        /// Inicijalni konstruktor koji postavlja poziciju svijetla i parametre svih
        /// objekata u sceni.
        /// </summary>
        /// <param name="lightPosition">pozicija svijetla</param>
        /// <param name="numberOfObjects">broj objekata u sceni</param>
        /// <param name="sphereParameters">parametri svih kugli</param>
        public Scene(Point lightPosition, int numberOfObjects, SphereParameters[] sphereParameters)
        {
            this.lightPosition = lightPosition;
            this.numberOfObjects = numberOfObjects;
            sphere = new Sphere[numberOfObjects];
            for (int i = 0; i < numberOfObjects; i++)
            {
                sphere[i] = new Sphere(sphereParameters[i]);
            }
        }

        /// <summary>
        /// Metoda provjerava da li postoji sjena na tocki presjeka. Vraca true ako
        /// se zraka od mjesta presjeka prema izvoru svjetlosti sjece s nekim objektom.
        /// </summary>
        /// <param name="intersection">tocka presjeka</param>
        /// <returns>true ako postoji sjena u tocki presjeka, false ako ne postoji</returns>
        private bool shadow(Point intersection)
        {
            Ray shadowRay = new Ray(intersection, lightPosition);

            for (var i = 0; i < numberOfObjects; i++)
            {
                if (sphere[i].intersection(shadowRay))
                {
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// Metoda koja pomocu pracenja zrake racuna boju u tocki presjeka. Racuna se
        /// osvjetljenje u tocki presjeka te se zbraja s doprinosima osvjetljenja koje
        /// donosi reflektirana i refraktirana zraka.
        /// </summary>
        /// <param name="ray">pracena zraka</param>
        /// <param name="depth">dubina rekurzije</param>
        /// <returns>vektor boje u tocki presjeka</returns>
        public ColorVector traceRay(Ray ray, int depth)
        {
            if (depth > MAXDEPTH)
            {
                return new ColorVector(0, 0, 0);    // crna boja
            }

            int index = -1;
            // pronalazak najblizeg presjeka zrake R sa scenom
            for (int i = 0; i < numberOfObjects; i++)
            {
                if (sphere[i].intersection(ray))
                {
                    if (index == -1)
                    {
                        index = i;
                    }
                    else if (ray.getStartingPoint().getDistanceFrom(sphere[index].getIntersectionPoint()) > ray.getStartingPoint().getDistanceFrom(sphere[i].getIntersectionPoint()))
                    {
                        index = i;
                    }
                }
            }

            var color = new ColorVector(0, 0, 0);
            // nema presjeka
            if (index == -1)
            {
                return new ColorVector(backgroundColors.getRed(), backgroundColors.getGreen(), backgroundColors.getBlue());
            }
            else
            {
                var intersection = sphere[index];

                // Ambijentno ( Ia * ka )
                var ka = intersection.getKa();
                double red = ambientLight.getRed() * ka.getRedParameter();
                double green = ambientLight.getGreen() * ka.getGreenParameter();
                double blue = ambientLight.getBlue() * ka.getBlueParameter();
                color = color.add(new ColorVector((float)red, (float)green, (float)blue));

                var point = intersection.getIntersectionPoint();

                var L = new Vector(point, lightPosition);
                L.normalize();
                var N = intersection.getNormal(point);
                var LN = L.dotProduct(N);

                var R = L.getReflectedVector(N);
                R.normalize();
                var V = new Vector(point, ray.getStartingPoint());
                V.normalize();
                var RV = R.dotProduct(V);

                if (!shadow(point))
                {
                    // Difuzno  ( Ii * kd * (L*N) )
                    var kd = intersection.getKd();
                    red = light.getRed() * kd.getRedParameter() * LN;
                    green = light.getGreen() * kd.getGreenParameter() * LN;
                    blue = light.getBlue() * kd.getBlueParameter() * LN;
                    color = color.add(new ColorVector((float)red, (float)green, (float)blue));


                    // Spekularno  ( Ii * ks * (R*V)^n )
                    var ks = intersection.getKs();
                    float n = intersection.getN();
                    red = light.getRed() * ks.getRedParameter() * Math.Pow(RV, n);
                    green = light.getGreen() * ks.getGreenParameter() * Math.Pow(RV, n);
                    blue = light.getBlue() * ks.getBlueParameter() * Math.Pow(RV, n);
                    color = color.add(new ColorVector((float)red, (float)green, (float)blue));
                }

                var RRefl = V.getReflectedVector(N);
                RRefl.normalize();

                // Reflektirano  ( Irefl * krefl )
                var RReflRay = traceRay(new Ray(point, RRefl), depth + 1);
                color = color.add(RReflRay.multiple(intersection.getReflectionFactor()));


                double b = intersection.getNi();
                if (V.dotProduct(N) < 0)
                {
                    N = N.multiple(-1);
                    b = 1.0 / b;
                }
                var RRefr = V.getRefractedVector(N, b);
                RRefr.normalize();

                // Refraktirano  ( Irefr * krefr )
                var RRefrRay = traceRay(new Ray(point, RRefr), depth + 1);
                color = color.add(RRefrRay.multiple(intersection.getRefractionFactor()));

                color.correct();
                return color;
            }
        }
    }
}