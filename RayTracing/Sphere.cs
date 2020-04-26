using System;

namespace raytracing
{
    /// <summary>
    /// Klasa predstavlja kuglu u prostoru. Nasljeduje apstraktnu klasu Object. Kugla
    /// je odredena svojim polozajem, radijusom, bojom, parametrima materijala i
    /// udjelima pojedninih zraka (osnovne, odbijene i lomljene).
    /// </summary>
    public class Sphere:Object
    {
        private double radius;
        const double Epsilon = 0.0001;
        private Point IntersectionPoint;

        /// <summary>
        /// Inicijalni konstruktor koji postavlja sve parametre kugle. Za prijenos
        /// parametara koristi se pomocna klasa SphereParameters.
        /// </summary>
        /// <param name="sphereParameters">parametri kugle</param>
        public Sphere ( SphereParameters sphereParameters )
            : base(sphereParameters.getCenterPosition(), sphereParameters.getRaysContributions(),
                sphereParameters.getMaterialParameters(), sphereParameters.getN(),
                sphereParameters.getNi())
        {
            this.radius = sphereParameters.getRadius();
        }

        /// <summary>
        /// Metoda ispituje postojanje presjeka zrake ray s kuglom. Ako postoji presjek
        /// postavlja tocku presjeka IntersectionPoint, te
        /// vraca logicku vrijednost true.
        /// </summary>
        /// <param name="ray">zraka za koju se ispituje postojanje presjeka sa kuglom</param>
        /// <returns>logicku vrijednost postojanja presjeka zrake s kuglom</returns>
        public override bool intersection ( Ray ray )
        {
            var P = ray.getStartingPoint();
            var C = getCenterPosition();
            var PC = new Vector(P, C);

            // provjera kuta između vektora smjera zrake Direction i vektora PC
            double alpha = ray.getDirection().getAngle(PC); // radijani
            if ((alpha * 180.0 / Math.PI) > 90)
            {
                return false;
            }

            // provjera udaljenosti zrake od sredista kugle (d = |PC| * sin(alpha) )
            double d = Math.Abs(PC.getLength()) * Math.Sin(alpha);
            if (d > radius)
            {
                return false;
            }

            // udaljenost hvatista zrake P od tocke D ( |PD| = sqrt(|PC^2| - d^2) ) 
            double PD = Math.Sqrt(Math.Abs(Math.Pow(PC.getLength(), 2)) - d * d);
            // udaljenost hvatista zrake P od blizeg presjeka ( |PBliziPresjek| = |PD| - sqrt(r^2 - d^2) )
            double PBliziPresjek = Math.Abs(PD) - Math.Sqrt(radius * radius - d * d);

            if (Math.Abs(PBliziPresjek) <= (0 + Epsilon))
            {
                PBliziPresjek = Math.Abs(PD) + Math.Sqrt(radius * radius - d * d);
            }

            IntersectionPoint = new Point(ray.getStartingPoint(), ray.getDirection(), PBliziPresjek);
            return true;
        }

        /// <summary>
        /// Vraca tocku presjeka kugle sa zrakom koja je bliza pocetnoj tocki zrake.
        /// </summary>
        /// <returns>tocka presjeka zrake s kuglom koja je bliza izvoru zrake</returns>
        public override Point getIntersectionPoint ()
        {
            return IntersectionPoint;
        }

        /// <summary>
        /// Vraca normalu na kugli u tocki point
        /// </summary>
        /// <param name="point">point na kojoj se racuna normala na kugli</param>
        /// <returns>normal vektor normale</returns>
        public override Vector getNormal ( Point point )
        {
            var ray = new Ray(centerPosition, point);
            return ray.getDirection();
        }
    }
}